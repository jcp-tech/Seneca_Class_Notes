from sklearn.datasets import load_diabetes
import pandas as pd

diabetes=load_diabetes()
diabetes

print(diabetes.keys())

print(diabetes.DESCR)

print(diabetes.feature_names) # these are the column names

df=pd.DataFrame(diabetes.data, columns=diabetes.feature_names)
df

df.shape

df.head()

df.columns

df.dtypes

df.loc[1]

df.loc[[1,99,149]]

df.iloc[[1,99]]

df.loc[:, 'bmi']

df.loc[:, ['bmi','bp']]

df.iloc[-1]

df.iloc[-2:,:]

subDf_q12 = df.loc[:10,["age","bmi","bp"]]
subDf_q12.head()

subDf_q13 = df.iloc[:,:6]
subDf_q13.head()

subDf_q14 = df.iloc[:,2:5]
subDf_q14.head()

subDf_q15 = df.iloc[:,:5:2] # Every other meaning Skip.
subDf_q15.head()

print(df.loc[9,"age"])

df.loc[[99,199,299],"bmi"]

# URL of the data
url = "https://www4.stat.ncsu.edu/~boos/var.select/diabetes.tab.txt"

# Import the data into a DataFrame
new_df = pd.read_csv(url, sep='\t')

new_df.rename(columns={'AGE': 'age', 'SEX': 'sex', 'BMI': 'bmi', 'BP': 'bp', 'S1': 's1', 'S2': 's2', 'S3': 's3', 'S4': 's4', 'S5': 's5', 'S6': 's6'}, inplace=True)
new_df

new_df.loc[new_df['age'] > 50]

new_df.loc[new_df['sex'] == 2, ['bmi', 'bp']] # Assuming Number 2 is the Standard for the Female in column 'sex'

ageSeries = new_df['age']
ageSeries

print("Min =",ageSeries.min())
print("Max =",ageSeries.max())
print("Mean =",ageSeries.mean())

ageSeries.info() # ageSeries.describe()

subDf_q23 = new_df.loc[ageSeries<ageSeries.mean(),"age"] 
# new_df.loc[ageSeries<ageSeries.mean()]['age']
# new_df[ageSeries<ageSeries.mean()]['age']
subDf_q23

ageSeries.sort_values() # sorted(ageSeries)

ageSeries.unique()

