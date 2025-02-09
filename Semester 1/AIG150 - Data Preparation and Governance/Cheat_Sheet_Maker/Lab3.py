import pandas as pd
customer = pd.read_csv('customer_demographics.csv')
customer.dtypes

customer.head() # .head(5)

customer['customer_id'] = customer['customer_id'].astype('category')
customer.dtypes

customer.columns

customer.columns = customer.columns.str.lower()
customer.dtypes

if "1001036" not in customer['customer_id'].cat.categories: # Checking if the Category exists
    customer['customer_id'] = customer['customer_id'].cat.add_categories("1001036") # Adding the Category if it does not exist
customer.loc[0:1, 'customer_id'] = "1001036"
customer.head(2)

import re
p = re.compile(pattern=r'^\d{6}$') # Regular expression pattern which matches a 6 digit number
match = p.match(customer.iloc[0, 1]) # Match the pattern with the first element of the column
print(match)

customer_id_m = list(map(p.match, customer['customer_id'].astype(str)))

for i, match in enumerate(customer_id_m):
    if not match:
        print(f"customer id {customer.iloc[i, 1]} in row {i + 1} : does not match the pattern")
    # else:
    #     print(f"customer id {customer.iloc[i, 1]} in row {i + 1} : matches the pattern")

list(customer['education'].unique())

customer['education_lowercase'] = customer['education'].str.lower()
list(customer['education_lowercase'].unique())

customer['education'] = customer['education_lowercase']
if 'education_lowercase' in customer.columns:
    customer.drop(columns=['education_lowercase'], inplace=True)
customer.head()

customer['job'].value_counts()

customer['job'].replace('ADMINISTRATION', 'admin.', inplace=True)
customer['job'].value_counts()

customer['mariage_status'] = customer['marital'].str.lower().str[:3]
customer.value_counts('mariage_status') # customer['mariage_status'].value_counts()

customer['marital_b'] = customer['mariage_status'].replace({'sin': 0, 'mar': 1, 'div': 2}) # customer['marital'].replace({'single': 0, 'married': 1, 'divorced': 2})
customer.value_counts('marital_b') # customer['marital_b'].value_counts()

# import numpy as np
customer.loc[customer['age'] < 35, 'age_group'] = 'young'
customer.loc[(customer['age'] >= 35) & (customer['age'] <= 55), 'age_group'] = 'middle'
customer.loc[customer['age'] > 55, 'age_group'] = 'senior'
# customer['age_group'] = np.select([
#     (customer['age'] < 35),
#     (customer['age'] >= 35) & (customer['age'] <= 55),
#     (customer['age'] > 55)
# ], ['young', 'middle', 'senior'], default='unknown')
customer.value_counts('age_group')

import kagglehub
path = kagglehub.dataset_download("arunjangir245/boston-housing-dataset") # Download latest version
boston = pd.read_csv(path+"/BostonHousing.csv")
boston.head()

quantiles = boston.quantile([0, 0.25, 0.5, 0.75, 1]) # 25th, 50th and 75th percentile in the data
quantiles

boston.describe()

boston.shape

boston_binned = boston.copy() # Copying the DF to Manupulate an Isolated Copy without affecting the Original Table
for col in boston.columns: # for col in boston.select_dtypes(include=['float64', 'int64']).columns:
    if boston[col].dtype != 'float64' and boston[col].dtype != 'int64':
        continue
    boston_binned[col] = pd.qcut(boston[col], q=4, duplicates='drop') # , labels=False
boston_binned.head() # .head(5)

boston_binned['crim'].value_counts()

# Create a crosstab between each Series table which shows the frequency of each combination of values in the two Columns.
crossTabDf = pd.crosstab(boston_binned['crim'], boston_binned['medv']) # NOTE: Can Crossverify each value with `boston_binned.loc[(boston_binned['crim'] == 3) & (boston_binned['medv'] == 3)]`
pd.DataFrame(crossTabDf) # Making with pd.DataFrame to make it look better in Jupyter Printing.

import matplotlib.pyplot as plt
import seaborn as sns
# Plot the heatmap
plt.figure(figsize=(10, 8))
sns.heatmap(crossTabDf, annot=True, cmap='coolwarm', fmt='d')
plt.title('Relationship between CRIM and MEDV')
plt.xlabel('MEDV')
plt.ylabel('CRIM')
plt.show()

from scipy.stats import chi2_contingency

# Calculate the chi-square test statistic and p-value
chi2, p, dof, expected = chi2_contingency(crossTabDf)

print(f"Chi-square test statistic: {chi2}")
print(f"P-value: {p}")
print(f"Degrees of freedom: {dof}")
print("Expected frequencies:") # , expected

expected # NOTE: I do not Understand this Question & had to refer to GPT for the Soltuion.

boston_binned['medv'].value_counts()

palette = {
    str(key) # Convertion to String cause if we dont use Label when using qcut, it will become a 'pandas._libs.interval.Interval'
    :value 
        for key, value in 
            zip(
                sorted(boston_binned['medv'].unique(), key=lambda x: x.left),
                ['red', 'blue', 'green', 'purple'] # Since we did qcut with 4 Quantiles, im taking 4 colors.
            )
}
palette # Printing the Dictionary

def is_in_interval(interval, number):
    # Remove parentheses or brackets and split into lower and upper bounds
    lower_inclusive = interval.startswith('[')
    upper_inclusive = interval.endswith(']')
    lower, upper = map(float, interval.strip('()[]').split(','))
    
    # Check if the number is within the interval
    return ((lower <= number if lower_inclusive else lower < number) and
            (number <= upper if upper_inclusive else number < upper))

plt.figure(figsize=(10, 8)) # Plotting scatter chart

for interval in sorted(boston_binned['medv'].unique()):
    subset = boston[boston['medv'].apply(lambda x: is_in_interval(str(interval), x))] # boston[boston['medv'].apply(lambda x: x in interval)]
    KEY = palette[str(interval)] # list(palette.keys())[interval]
    plt.scatter(subset['dis'], subset['medv'], label=str(interval), color=KEY)
    plt.legend(title='MEDV Intervals')

plt.xlabel('DIS')
plt.ylabel('MEDV')
plt.title('Scatter plot of MEDV vs DIS')
plt.show()

# Interpreting the Scatter Plot:
# The Scatter Plot shows the relationship between the 'DIS' and 'MEDV' columns.
# When Looking at the Diagram we can see that there is a Weak Positive Correlation between the 'DIS' and 'MEDV' columns.
# This can be Verified by the fact that as the 'DIS' increases, the 'MEDV' also increases, but the increase is not very significant.
# Below Is the Code for Correlation Matrix to Verify the Correlation between the Columns.
corrMatrix = boston[['dis', 'medv']].corr()
corrMatrix # Now when we look at the Output it's `0.249929` which is a Weak Positive Correlation.

# NOTE: The Q22 is same as Q21, so please refer to the Above Plot for the Scatter Chart & it's Interpretation.

from pandas.plotting import scatter_matrix
columns_to_plot = ['crim', 'zn', 'indus', 'rm', 'medv']
scatter_matrix(boston[columns_to_plot], figsize=(10, 10), diagonal='kde', alpha=0.2)
plt.show()

