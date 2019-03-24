import subprocess

#build = 'debug'
build = 'release'
maps = ('30x30.map', )
timeout_seconds = 60
seed = 123

for num_agents in range(3, 30):
    for map in maps:
        for i in range(100):
            for split_strategy in ('WIDTH', ):  # Looks like the best one
                cmd = f'./cmake-build-{build}/disjoint_CBSH -m {map} -a agents/{map}_{num_agents}_{i}.agents ' \
                      f'-o {map}_output.csv -k {num_agents} -p {split_strategy} --screen 0 --seed {seed} ' \
                      f'--cutoffTime={timeout_seconds}'
                print(cmd)
                subprocess.check_call(cmd, shell=True)
