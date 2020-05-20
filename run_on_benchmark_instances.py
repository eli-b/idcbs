from itertools import product
from multiprocessing import Process, Queue, Pool, Manager, Lock
import subprocess
from os.path import exists, join, abspath
from contextlib import nullcontext
from os import getcwd, remove
import time
import csv
import logging


logging.basicConfig(filename='log.txt', datefmt='%Y-%m-%dT%H:%M:%S', format='%(asctime)s %(levelname)s:%(name)s:%(message)s')

#build = 'debug'
build = 'release'
timeout_seconds = 3600
in_container = True
in_multiprocessing_pool = True
mem_limit = '8g'
seed = 123
use_heuristic = True
focal_w = 1#.50
child_pref_budget = 5
max_child_pref_options = 20
prefer_f_cardinal = False
prefer_goal_conflicts = False
skip_according_to_output_file = True
num_processes = 16

agents_dir = abspath('agents')
maps_dir = abspath('maps')
scen_dir = abspath('scen')
# output_file_name = 'benchmark_cbsh_no_lpa.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_timeout_3600.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_prefer_goal_conflicts.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic_prefer_goal_conflicts.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic_prefer_f_cardinal.csv'
# output_file_name = 'benchmark_cbsh_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_50_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_05_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_01_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_00_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600-in-container.csv'
output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600-in-container-with-conflict-choice-fix.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600-in-container-with-conflict-choice-fix2.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600-in-container-with-conflict-choice-fix3.csv'
# output_file_name = 'benchmark_ecbsh_1_05_no_lpa.csv'
# output_file_name = 'benchmark_ecbsh_1_01_no_lpa.csv'
# output_file_name = 'benchmark_cbsh_lpa_up_and_down_with_lpmdd_and_path_repair_goal_conflict_heuristic.csv'
# output_file_name = 'benchmark_cbsh_lpa_up_and_down_5_with_lpmdd_and_path_repair_goal_conflict_heuristic_prefer_f_cardinals.csv'
# output_file_name = 'dao_cbsh_no_lpa_fcardinal.csv'
#output_file_name = 'dao_cbsh_no_lpa_latest_conflict.csv'
#output_file_name = 'dao_cbsh_no_lpa_up_and_down_0_budget.csv'
#output_file_name = 'dao_cbsh_no_lpa_up_and_down_0_budget_latest_conflict.csv'
#output_file_name = 'dao_cbsh_no_lpa_up_and_down_5_budget.csv'
#output_file_name = 'dao_cbsh_no_lpa_up_and_down_5_budget_latest_conflict.csv'
#output_file_name = 'dao_cbsh_no_lpa_up_and_down_5_budget_min_tree_distance_20_options_.csv'
#output_file_name = 'dao_cbsh_lpa.csv'
#output_file_name = 'dao_cbsh_lpa_up_and_down_0_budget.csv'
#output_file_name = 'dao_cbsh_lpa_up_and_down_0_budget_not_latest_conflict.csv'
#output_file_name = 'dao_cbsh_lpa_up_and_down_5_budget.csv'
#output_file_name = 'dao_cbsh_lpa_up_and_down_20_budget.csv'
#output_file_name = 'dao_cbsh_lpa_up_and_down_5_budget_not_latest_conflict.csv'
#output_file_name = 'dao_ecbs_no_h_1_05_lpa_up_and_down_5_budget.csv'
#output_file_name = 'dao_ecbsh_1_05_lpa_up_and_down_5_budget.csv'
#output_file_name = 'dao_ecbs_no_h_1_05_no_lpa_up_and_down_5_budget.csv'
#output_file_name = 'dao_ecbsh_1_05_no_lpa_up_and_down_5_budget.csv'
# output_file_name = 'dao_ecbsh_1_05_no_lpa_up_and_down_5_budget_latest_conflict.csv'
#output_file_name = 'dao_ecbsh_1_05_lpa.csv'
#output_file_name = 'dao_ecbsh_1_05_no_lpa.csv'
#output_file_name = 'dao_idcbsh_no_lpa.csv'
#output_file_name = 'dao_idcbsh_no_lpa_latest_conflict.csv'
#output_file_name = 'dao_idcbsh_lpa.csv'
#output_file_name = 'dao_ecbs_no_h_1_05_no_lpa.csv'
output_dir = getcwd()
output_file_path = join(output_dir, output_file_name)
# executable_name = "ECBSH_no_lpa"
# executable_name = "ECBSH_no_lpa_goal_conflict_heuristic"
#executable_name = "ECBSH_no_lpa_latest_conflict"
#executable_name = "ECBSH_no_lpa_up_and_down"
# executable_name = "ECBSH_no_lpa_up_and_down_latest_conflict"
# executable_name = "ECBSH_lpa"
# executable_name = "ECBSH_lpa_not_latest_conflict"
#executable_name = "ECBSH_lpa_up_and_down"
# executable_name = "ECBSH_lpa_up_and_down_with_lpmdd_and_path_repair"
# executable_name = "ECBSH_lpa_up_and_down_with_lpmdd_and_path_repair_goal_conflict_heuristic"
#executable_name = "ECBSH_lpa_up_and_down_not_latest_conflict"
# executable_name = "IDCBSH_no_lpa"
# executable_name = "IDCBSH_no_lpa_latest_conflict"
# executable_name = "IDCBSH_lpa"
# executable_name = "IDCBSH_lpa_not_latest_conflict"
executable_name = "IDECBSH_lpa_with_lpmdd_and_path_repair"
executable_path = join('/lpa', executable_name)

map_names = (
    'ost003d',
    'den502d',
    'den520d',
    'brc202d',
    'empty-8-8',
    'empty-16-16',
    'empty-32-32',
    'empty-48-48',
    'random-32-32-10',
    'random-32-32-20',
    'random-64-64-10',
    'random-64-64-20',
    'maze-128-128-1',
    'maze-128-128-2',
    'maze-128-128-10',
    'maze-32-32-2',
    'room-32-32-4',
    'room-64-64-8',
    'room-64-64-16',
    'den312d',
    'orz900d',
    'ht_chantry',
    'ht_mansion_n',
    'lak303d',
    'lt_gallowstemplar_n',
    'w_woundedcoast',
    'Berlin_1_256',
    'Boston_0_256',
    'Paris_1_256',
    'warehouse-10-20-10-2-1',
    'warehouse-10-20-10-2-2',
    'warehouse-20-40-10-2-1',
    'warehouse-20-40-10-2-2',
)
# USC maps (with .agents files):
# 'kiva_0.map',
# 'roundabout_2.map',
# 'maze_3.map',
# 'maze_2.map',
# 'roundabout_3.map',
# 'roundabout_1.map',
# 'roundabout_4.map',
# 'roundabout_5.map',
# 'maze_1_2.map',
# 'maze_1.map',
# 'maze_1_3.map',
# 'maze_4.map',

last_scen_type = None
last_map_name = None
last_scen_index = None
last_num_agents = None
last_cost = None
if exists(output_file_path) and skip_according_to_output_file:
    with open(output_file_path) as f:
        output_csv = csv.DictReader(f)
        lines = list(output_csv)
        last_line = lines[-1]
        del output_csv
        del lines
        instance_file = last_line['instance']
        assert instance_file.endswith('.scen')
        instance_name = instance_file[:-len('.scen')]
        last_scen_type = instance_name.rsplit('-', 2)[-2]
        last_map_name = instance_name.rsplit('-', 2)[-3].partition('scen/')[2]
        last_scen_index = int(instance_name.rsplit('-', 2)[-1])
        last_num_agents = int(last_line['Num of Agents'])
        last_cost = int(last_line['Cost'])
        logging.info(f'Existing file ends with:'
                     f'instance={instance_file} - last_scen_type={last_scen_type}, '
                     f'last_map_name={last_map_name}, last_scen_index={last_scen_index}')


def job(general_cmd, num_agents_start, mem_limit, scen_file_name, output_queue, lock):
    import subprocess
    import time
    for num_agents in range(num_agents_start, 2000):  # We stop on the first failure
        tmp_path = f'XXX{num_agents}XXX{scen_file_name}XXX{hash(general_cmd)}.tmp'  # Can't use tempfile because its files are only readable and writable by the same UID
        cmd = general_cmd % (num_agents, tmp_path)
        try:
            with lock:
                print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd)
            start_time = time.time()
            subprocess.check_call(cmd, shell=True)
            with lock:
                print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd + ' finished')
            with open(tmp_path) as f:
                output_queue.put(f.read())
        except subprocess.CalledProcessError as e:
            if e.returncode == 1:  # Solution not found - probably due to a timeout. No point in adding more agents
                with lock:
                    print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd + ' finished')
                with open(tmp_path) as f:
                    output_queue.put(f.read())
                break
            elif e.returncode == 137:  # Killed by the cgroup's OOM killer
                with lock:
                    print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd + ' finished')
                output_queue.put('-2,' + '=NA(),' * 23 + f'{time.time() - start_time},=NA(),{mem_limit},same as above,/scen/{scen_file_name},{num_agents}\n')
                break
            else:
                raise
        finally:
            remove(tmp_path)


def writer_job(output_queue, output_file_name, stop_token, lock):
    import time
    with lock:
        print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + f'writer started. Writing to {output_file_name}')
    wrote_header = False
    with open(output_file_name, mode='w') as f:
        while True:
            with lock:
                print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + 'writer is getting next item from the queue')
            lines = output_queue.get()
            with lock:
                print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + f'writer got an output of size {len(lines)} from the queue and is writing it to file')
            if lines == stop_token:
                return
            if not wrote_header:
                f.write(lines)
            else:
                header, newline, rest = lines.partition('\n')
                f.write(rest)
            f.flush()


pool = lock = nullcontext()
result_objects = []
queue = stop_token = None
if in_multiprocessing_pool:
    pool = Pool(processes=num_processes)
    manager = Manager()
    queue = manager.Queue()  # A manager allows sharing a queue between multiple worker processes
    lock = manager.Lock()  # A manager allows sharing a lock between multiple worker processes
    stop_token = '!!!STOP!!!'
with pool:
    for split_strategy, scen_index, map_name, scen_type in product(
            (
                    #'WIDTH',
                    #'MVC_BASED',
                    'NON_DISJOINT',
            ), range(1, 26), map_names,
            (
                    'even',
                    'random',
            )):
        #            for split_strategy in ('WIDTH', ):  # Looks like the best one

        num_agents_start = 2

        # Skip instances we've already done. TODO: support multiple split strategies
        if last_scen_type is not None:
            if scen_type == last_scen_type and map_name == last_map_name and scen_index == last_scen_index:
                last_scen_type = None
                last_map_name = None
                last_scen_index = None
                num_agents_start = last_num_agents + 1
                last_num_agents = None
                if last_cost >= 0:
                    logging.info(f'Resuming the run, starting from {num_agents_start}')
                else:
                    num_agents_start = 2
                    logging.info(f'Skipping ({scen_index},{map_name},{scen_type})')
                    logging.info(f'Resuming the run')
                    continue
            else:
                logging.info(f'Skipping ({scen_index},{map_name},{scen_type})')
                continue

        map_file_name = f'{map_name}.map'
        map_file_path = join(maps_dir, map_file_name)
        if not exists(map_file_path):
            with lock:
                logging.warning(f'No map file {map_file_path}')
            continue
        scen_file_name = f'{map_name}-{scen_type}-{scen_index}.scen'
        scen_file_path = join(scen_dir, scen_file_name)
        if not exists(scen_file_path):
            with lock:
                logging.warning(f'No scenario file {scen_file_path}')
            continue

        if in_multiprocessing_pool:
            cmd = f'docker run --rm -it --net=host --memory={mem_limit} --memory-swap={mem_limit} -v {scen_dir}:/scen ' \
                  f'-v {maps_dir}:/maps -v {output_dir}:/output ' \
                  f'eliboyarski/mapf:cbs-lpa ' \
                  f'{executable_path} -m "/maps/{map_file_name}" ' \
                  f'-a "/scen/{scen_file_name}" ' \
                  f'-k %s ' \
                  f'-o /output/%s -p {split_strategy} --screen 0 --seed {seed} ' \
                  f'--cutoffTime={timeout_seconds} --verbosity 0 --heuristic {"CG" if use_heuristic else "NONE"} ' \
                  f'--focalW {focal_w} --childPrefBudget {child_pref_budget} --maxChildPrefOptions {max_child_pref_options} ' \
                  f'--prefer_f_cardinal {1 if prefer_f_cardinal else 0} ' \
                  f'--prefer_goal_conflicts {1 if prefer_goal_conflicts else 0}'
            with lock:
                logging.info(f'sending job {scen_file_name}')
            result_objects.append(pool.apply_async(job, (cmd, num_agents_start, mem_limit, scen_file_name, queue, lock)))
        else:
            for num_agents in range(num_agents_start, 2000):  # We stop on the first failure
                # GLOG_logtostderr=1  ./cmake-...
                # docker run --memory-swap=10g to avoid swapping

                if in_container:
                    cmd = f'docker run --rm -it --net=host --memory={mem_limit} --memory-swap={mem_limit} -v {scen_dir}:/scen ' \
                          f'-v {maps_dir}:/maps -v {output_dir}:/output ' \
                          f'search/mapf:cbs-lpa ' \
                          f'{executable_path} -m "/maps/{map_file_name}" ' \
                          f'-a "/scen/{scen_file_name}" ' \
                          f'-k {num_agents} ' \
                          f'-o /output/{output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
                          f'--cutoffTime={timeout_seconds} --verbosity 0 --heuristic {"CG" if use_heuristic else "NONE"} ' \
                          f'--focalW {focal_w} --childPrefBudget {child_pref_budget} --maxChildPrefOptions {max_child_pref_options} ' \
                          f'--prefer_f_cardinal {1 if prefer_f_cardinal else 0} ' \
                          f'--prefer_goal_conflicts {1 if prefer_goal_conflicts else 0}'
                else:
                    cmd = f'./{executable_name} -m "maps/{map_file_name}" ' \
                          f'-a "scen/{scen_file_name}" ' \
                          f'-k {num_agents} ' \
                          f'-o {output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
                          f'--cutoffTime={timeout_seconds} --verbosity 0 --heuristic {"CG" if use_heuristic else "NONE"} ' \
                          f'--focalW {focal_w} --childPrefBudget {child_pref_budget} --maxChildPrefOptions {max_child_pref_options} ' \
                          f'--prefer_f_cardinal {1 if prefer_f_cardinal else 0} ' \
                          f'--prefer_goal_conflicts {1 if prefer_goal_conflicts else 0}'

                with lock:
                    logging.info(f'Running {cmd}')
                start_time = time.time()
                try:
                    subprocess.check_call(cmd, shell=True)
                except subprocess.CalledProcessError as e:
                    if e.returncode == 1:  # Solution not found - probably due to a timeout. No point in adding more agents
                        break
                    elif e.returncode == 137:  # Killed by the cgroup's OOM killer
                        with open(output_file_path, 'a') as f:
                            f.write('-2,' + '=NA(),' * 23 + f'{time.time() - start_time},=NA(),{mem_limit},same as above,/scen/{scen_file_name},{num_agents}\n')
                        break
                    else:
                        # 139 is SIGSEGV, BTW
                        raise
    if in_multiprocessing_pool:
        writer_process = Process(target=writer_job, args=(queue, output_file_name, stop_token, lock))
        writer_process.start()

        # Wait for all jobs to finish
        for result_object in result_objects:
            try:
                result_object.get()  # Blocking
            except Exception as e:
                logging.exception('Ignoring the following exception from a pool worker')

        queue.put(stop_token)
        writer_process.join()
        writer_process.close()
