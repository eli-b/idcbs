from itertools import product
import subprocess
from os.path import basename, exists, join, abspath
from os import getcwd
import time

#build = 'debug'
build = 'release'
timeout_seconds = 60
seed = 123

instances_dir = f'/media/eli/OS/Documents and Settings/User/mapf/instances/'
agents_dir = abspath('agents')
maps_dir = abspath('maps')
output_file_name = 'dao.csv'
#output_file_name = 'dao_cbs_no_lpa.csv'
#output_file_name = 'dao_idcbs_no_lpa.csv'
output_dir = getcwd()
output_file_path = join(output_dir, output_file_name)
executable_path = f'/lpa/cmake-build-{build}/disjoint_CBSH'
#executable_path = '/lpa/disjoint_CBSH_no_lpa'
#executable_path = '/lpa/disjoint_IDCBSH_no_lpa'
map_names = ('den502d', 'ost003d', 'brc202d', 'den520d')

skip = 0
if exists(output_file_path):
    with open(output_file_path) as f:
        skip = len(f.readlines())#- 1 no header to the csv

for map_name, num_agents, i in product(map_names, range(5, 100, 5), range(100)):
#            for split_strategy in ('WIDTH', ):  # Looks like the best one
    for split_strategy in ('NON_DISJOINT', ):
        my_instance_file_path = join(instances_dir, f'{map_name}-{num_agents}-{i}')
        #i += 1
        #if i-1 < num_existing_lines:
        #   continue
        agents_file_name = f'{basename(my_instance_file_path)}.agents'
        agents_file_path = join(agents_dir, agents_file_name)
        map_file_name = f'{basename(my_instance_file_path)}.map'
        map_file_path = join(maps_dir, map_file_name)
        if not exists(map_file_path) or not exists(agents_file_path):
            if not exists(my_instance_file_path):
                continue
            with open(my_instance_file_path) as f:
                my_instance_lines = f.readlines()
            instance_id, map_name = my_instance_lines[0].split(',')
            instance_id = int(instance_id)
            grid_line = my_instance_lines[1]
            x, y = [int(part) for part in my_instance_lines[2].split(',')]
            with open(map_file_path, 'w') as f:
                f.write(my_instance_lines[2])
                for j in range(x):
                    f.write(my_instance_lines[j+3])
            agents_line = my_instance_lines[3+x]
            num_agents_line = my_instance_lines[3+x+1]
            with open(agents_file_path, 'w') as f:
                f.write(num_agents_line)
                for line in my_instance_lines[3+x+2:]:
                    f.write(','.join(line.split(',')[1:]))

        if skip > 0:
            skip -= 1
            continue
        # GLOG_logtostderr=1  ./cmake-...
        # docker run --memory-swap=10g to avoid swapping
        cmd = f'docker run --rm -it --memory=8g --memory-swap=8g -v {agents_dir}:/agents ' \
              f'-v {maps_dir}:/maps -v {output_dir}:/output ' \
              f'search/mapf:cbs-lpa ' \
              f'{executable_path} -m "/maps/{map_file_name}" ' \
              f'-a "/agents/{agents_file_name}" ' \
              f'-o /output/{output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
              f'--cutoffTime={timeout_seconds} --verbosity 0'
        #rf'/bin/bash -c "/usr/local/bin/time -f \"%M\" {executable_path} -m \"/maps/{map_file_name}\" ' \
        #rf'-a \"/agents/{agents_file_name}\" ' \
        #f'-o /output/{output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
        #f'--cutoffTime={timeout_seconds} --verbosity 0 2> >(tee --append /output/maxmem_{output_file_name})"'
        print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd)
        try:
            subprocess.check_call(cmd, shell=True)
        except subprocess.CalledProcessError as e:
            if e.returncode == 137:  # Killed by the OOM killer, IIRC, shouldn't happen now with the memory limit
                with open(output_file_path, 'a') as f:
                    f.write('=NA(),' * 10 + f'same as above,{agents_file_path}\n')
            else:
                raise
