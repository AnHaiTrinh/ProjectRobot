import pandas as pd
import os

#Change directory if needed
scenario = input("Enter scenario: ")
algorithms = os.listdir(scenario)
for algorithm in algorithms:
    df = pd.read_csv(scenario + '/' + algorithm, sep=' ', names=['Map', 'Length', 'Smoothness', 'Time', 'Err'])
    df = df.drop(['Time', 'Err'], axis=1)
    df['Map'] = df['Map'].transform(lambda x: x[:-1])
    df.to_excel(scenario + '.xlsx', sheet_name=algorithm)
    df.loc[len(df.index)] = ['Sum', df['Length'].sum(), df['Smoothness'].sum()]
    df.loc[len(df.index)] = ['Std', df['Length'].std(), df['Smoothness'].std()]
    df.to_excel(scenario + '.xlsx', sheet_name=algorithm)