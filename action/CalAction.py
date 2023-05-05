import os
import pandas as pd
pd.set_option('display.max_colwidth', None)
pd.set_option('display.max_rows', None)

# scenarios = ['dense', 'maze', 'room', 'trap']
scenario = input("Scenario: ")
algorithms = os.listdir(scenario)
for algorithm in algorithms:
    df = pd.read_csv(scenario + '/' + algorithm, sep=' ', names=['Map', 'Action', 'Time'])
    res = df.groupby(['Map', 'Action']).aggregate(['mean', 'sum'])
    res.to_excel(scenario + '.xlsx', sheet_name=algorithm)
