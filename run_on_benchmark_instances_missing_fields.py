from itertools import product
import subprocess
from os.path import basename, exists, join, abspath
from os import getcwd
import time
import csv

#build = 'debug'
build = 'release'
timeout_seconds = 3600
mem_limit = '8g'
seed = 123
use_heuristic = True
focal_w = 1#.01
child_pref_budget = 5
max_child_pref_options = 20
prefer_f_cardinal = False
prefer_goal_conflicts = False

agents_dir = abspath('agents')
maps_dir = abspath('maps')
scen_dir = abspath('scen')
# output_file_name = 'benchmark_cbsh_no_lpa.csv'
output_file_name = 'benchmark_cbsh_no_lpa_timeout_3600.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_prefer_goal_conflicts.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic_prefer_goal_conflicts.csv'
# output_file_name = 'benchmark_cbsh_no_lpa_goal_conflict_aware_heuristic_prefer_f_cardinal.csv'
# output_file_name = 'benchmark_cbsh_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_05_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_ecbsh_1_01_lpa_up_and_down_5_child_pref_budget_20_pref_options_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair.csv'
# output_file_name = 'benchmark_idcbsh_lpa_with_lpmdd_and_path_repair_timeout_3600.csv'
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
executable_name = "ECBSH_no_lpa"
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
# executable_name = "IDECBSH_lpa_with_lpmdd_and_path_repair"
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
    'maze-128-128-4',
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


with open(output_file_path) as f:
    output_csv = csv.DictReader(f)
    for line in output_csv:
        if line['Max Mem (kB)'] != '' and line['Wall Runtime'] != '#N/A':
            continue

        instance_file = line['instance']
        assert instance_file.endswith('.scen')
        instance_name = instance_file[:-len('.scen')]
        scen_type = instance_name.rsplit('-', 2)[-2]
        map_name = instance_name.rsplit('-', 2)[-3][len('/scen/'):]
        scen_index = int(instance_name.rsplit('-', 2)[-1])
        num_agents = int(line['Num of Agents'])
        print(f'found line with missing memory usage data or wall runtime:')
        print(f'instance={instance_file} - last_scen_type={scen_type}, last_map_name={map_name}, last_scen_index={scen_index}')

        split_strategy = 'NON_DISJOINT'
    # for , scen_index, map_name, scen_type in product(
    #         (
    #             #'MVC_BASED',
    #             ,
    #         ), range(1, 26), map_names, ('even', 'random')):
    #     #            for split_strategy in ('WIDTH', ):  # Looks like the best one
    #
    #     num_agents_start = 2

        map_file_name = f'{map_name}.map'
        map_file_path = join(maps_dir, map_file_name)
        if not exists(map_file_path):
            continue
        scen_file_name = f'{map_name}-{scen_type}-{scen_index}.scen'
        scen_file_path = join(scen_dir, scen_file_name)
        if not exists(scen_file_path):
            continue

        # GLOG_logtostderr=1  ./cmake-...
        # docker run --memory-swap=10g to avoid swapping

        cmd = f'docker run --rm -it --net=host --memory={mem_limit} --memory-swap={mem_limit} -v {scen_dir}:/scen ' \
              f'-v {maps_dir}:/maps -v {output_dir}:/output ' \
              f'search/mapf:cbs-lpa ' \
              f'{executable_path} -m "/maps/{map_file_name}" ' \
              f'-a "/scen/{scen_file_name}" ' \
              f'-k {num_agents} ' \
              f'-o /output/{output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
              f'--cutoffTime={timeout_seconds} --verbosity 0 --heuristic {1 if use_heuristic else 0} ' \
              f'--focalW {focal_w} --childPrefBudget {child_pref_budget} --maxChildPrefOptions {max_child_pref_options} ' \
              f'--prefer_f_cardinal {1 if prefer_f_cardinal else 0} ' \
              f'--prefer_goal_conflicts {1 if prefer_goal_conflicts else 0}'

        # cmd = f'./{executable_name} -m "maps/{map_file_name}" ' \
        #       f'-a "scen/{scen_file_name}" ' \
        #       f'-k {num_agents} ' \
        #       f'-o {output_file_name} -p {split_strategy} --screen 0 --seed {seed} ' \
        #       f'--cutoffTime={timeout_seconds} --verbosity 0 --heuristic {1 if use_heuristic else 0} ' \
        #       f'--focalW {focal_w} ' \
        #       f'--childPrefBudget {child_pref_budget} ' \
        #       f'--maxChildPrefOptions {max_child_pref_options} ' \
        #       f'--prefer_f_cardinal {1 if prefer_f_cardinal else 0} ' \
        #       f'--prefer_goal_conflicts {1 if prefer_goal_conflicts else 0}'

        print(time.strftime('%Y-%m-%dT%H:%M:%S: ') + cmd)
        start_time = time.time()
        try:
            subprocess.check_call(cmd, shell=True)
        except subprocess.CalledProcessError as e:
            if e.returncode == 1:  # Solution not found - probably due to a timeout. No point in adding more agents
                pass
            elif e.returncode == 137:  # Killed by the OOM killer
                with open(output_file_path, 'a') as f:
                    f.write('-2,' + '=NA(),' * 23 + f'{time.time() - start_time},=NA(),{mem_limit},same as above,/scen/{scen_file_name},{num_agents}\n')
            else:
                raise
