import os
import pandas as pd
pd.set_option('display.max_colwidth', None)
pd.set_option('display.max_rows', None)

scenarios = ['dense', 'maze', 'room', 'trap']
scenario = 'trap'
algorithms = os.listdir(scenario)
for algorithm in algorithms:
    df = pd.read_csv(scenario + '/' + algorithm, sep=' ', names=['Map', 'Action', 'Time'])
    res = df.groupby(['Map', 'Action']).mean()
    res.to_excel('output.xlsx', sheet_name=scenario)
