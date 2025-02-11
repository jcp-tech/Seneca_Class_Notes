import pandas as pd
customer = pd.read_csv('customer_demographics.csv')
customer.dtypes

customer.head()

customer['customer_id'] = customer['customer_id'].astype('category')
customer.dtypes

customer.columns = customer.columns.str.lower()
customer

# This question will not be included as a full mark - if you got this incorrect, you will get no marks lost.
# If you got this correct, I will count it as bonus marks

#since customer_id has been made into a category column, if we have a new category, we have to add it as shown below
customer['customer_id'] = customer['customer_id'].cat.add_categories([1001036])

# we can then use loc to update the values. If we tried doing this loc line, without adding the new customer_id
# as part of the category, we would get an error. This is a characteristic specific to categorical columns.
customer.loc[0:1, 'customer_id'] = 1001036
customer

# I did not show you how to create a pattern - I will give half marks if you
# were able to find you needed to use regular expression (regex) here.
# Full marks if you give me the answer below (ie. None, or false)

#importing regex, package name re
import re
#creating a pattern p, by using re.compile, and using regex notation to define what a 6-digit string looks like
p = re.compile(r'^\d{6}$')
#using p.match and passing the new customer_id we created, to see if it matches our pattern
print(p.match(customer.iloc[0,1].astype(str)))
# IT SHOULD NOT MATCH - our new customer_id is more than 6 digits long! Which is why our output is None
# try this out with a different row (ie. .iloc[7,1]) - this should give you a non-None answer!

'''
 for loops and mapping are concepts I did not teach either.
 Since these are fairly advanced in comparison to what I did teach, this will be treated as a bonus question.
'''

#repeat the same steps from Question 5
p = re.compile(r'^\d{6}$')
customer_id_m = list(map(p.match, customer['customer_id'].astype(str)))

#for loop will iterate through each row of our customer_id_m list, check if there is a None (like we saw in the answer anbove)
# and if there is a None, will print the error message
for i in range(len(customer_id_m)):
    if customer_id_m[i] == None:
        print(f'customer id {customer.iloc[i,1]} in row {i+1} : does not match the pattern')

print(customer['education'].unique())
# we see secondary appears twice - as all caps, and as lowercase
customer['education_lowercase'] = customer['education'].str.lower()
print(customer['education_lowercase'].unique())

customer['education'] = customer['education_lowercase']
customer = customer.drop(columns=['education_lowercase'])
customer.head()

print(customer['job'].value_counts())
# we see admin. and ADMINISTRATION, which should be the same value
customer['job'] = customer['job'].replace('ADMINISTRATION', 'admin.')
print(customer['job'].value_counts())

customer['marital'] = customer['marital'].str.lower()
customer['marital_first3'] = customer['marital'].str[:3]
customer.head()

customer['marital_b'] = customer['marital'].replace({'single':0, 'married':1, 'divorced':2})
customer.head()

# there are other ways to do this, I will accept any correct answer
customer.loc[customer['age'] < 35, 'age_group'] = 'young'
customer.loc[(customer['age'] >= 36) & (customer['age'] <= 55), 'age_group'] = 'middle'
customer.loc[customer['age'] > 55, 'age_group'] = 'senior'
customer

boston = pd.read_csv('BostonHousing.csv')
boston.head()

boston.quantile([0, 0.25, 0.5, 0.75, 1])

boston.describe()

print(boston.shape)

# bonus question - not going to affect your mark if you get it wrong
boston_binned = pd.DataFrame()
for column in boston.columns:
  if pd.api.types.is_numeric_dtype(boston[column]):
    boston_binned[column] = pd.qcut(boston[column], q=[0.00, 0.25, 0.5, 0.75, 1], duplicates='drop')

boston_binned.head()

boston_binned['crim'].value_counts()

#any reasonable comparison of the two categorical variables, I would accept.
#a table showing counts, a bar graph, etc
boston_binned.groupby(['crim', 'medv']).size()

import matplotlib.pyplot as plt

# Assuming boston_binned is your dataframe and 'crim' and 'medv' are categorical variables
pd.crosstab(boston_binned['crim'], boston_binned['medv']).plot(kind='bar', figsize=(10, 6))
plt.title('Relationship between CRIM and MEDV')
plt.xlabel('CRIM')
plt.ylabel('MEDV')
plt.show()


from scipy.stats import chi2_contingency

contingency_table = pd.crosstab(boston_binned['crim'], boston_binned['medv'])
chi2, p, dof, expected = chi2_contingency(contingency_table)

print(f"Chi-square statistic: {chi2}")
print(f"P-value: {p}")

boston_binned['medv'].value_counts()

palette = {
    '(4.999, 17.025]': 'red',
    '(17.025, 21.2]': 'blue',
    '(21.2, 25.0]': 'green',
    '(25.0, 50.0]': 'yellow'
}

print(palette)

# there were multiple ways to do the comparison here; I chose to do it with a mixture of the binned and not binned tables.
# I will accept either answer
palette = {pd.Interval(*map(float, k[1:-1].split(',')), closed='right'): v for k, v in palette.items()} # Split the string by comma, convert the values to float, and unpack them as arguments for pd.Interval

colors = [palette[interval] for interval in boston_binned['medv']]
scatter = boston.plot.scatter(x='dis', y='medv', c=colors, figsize=(10, 6))
scatter

#21 and 22 are the same question - regardless of your answer in 21, I will include 22 as a bonus point. :)

from pandas.plotting import scatter_matrix
scatter_matrix(boston[['crim', 'zn', 'indus', 'rm', 'medv']], figsize=(12, 12))
plt.show()

