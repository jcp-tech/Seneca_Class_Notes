import numpy as np
import pandas as pd

customer = pd.read_csv('customer.csv', index_col=0)
customer.head()

invoice = pd.read_csv('invoice.csv', index_col=0)
invoice.head()

invoice['CUS_CODE'].value_counts()

customer.loc[len(customer)] = [10020, 'Smith', 'James']
print(customer)

customer_invoice = pd.merge(customer, invoice, on='CUS_CODE', how='left')
customer_invoice

customer_invoice = pd.merge(customer, invoice, on='CUS_CODE', how='outer')
customer_invoice

customerInvoice = pd.merge(customer, invoice, on='CUS_CODE', how='inner')
customerInvoice

customerInvoice.to_csv('customerInvoice.csv')

population = pd.read_csv('https://raw.githubusercontent.com/jakevdp/data-USstates/master/state-population.csv')
area = pd.read_csv('https://raw.githubusercontent.com/jakevdp/data-USstates/master/state-areas.csv')
abbreviation = pd.read_csv('https://raw.githubusercontent.com/jakevdp/data-USstates/master/state-abbrevs.csv')

population.head()

area.head()

abbreviation.head()

abbreviation_population = pd.merge(abbreviation, population, left_on='abbreviation', right_on='state/region')
abbreviation_population.head()

abbreviation_population

abbreviation_population['state/region'].unique()

abbreviation_population['state'] = abbreviation_population['state'].fillna('Puerto Rico')
abbreviation_population['abbreviation'] = abbreviation_population['abbreviation'].fillna('PR')
abbreviation_population

result_df = pd.merge(abbreviation_population, area, left_on='state', right_on='state')
result_df

titanic = pd.read_csv('titanic.csv')
titanic.head()

# assuming that we wanted the average values of the survived column for each gender + class:
pivot_1 = titanic.pivot_table(index='sex', columns='class', values='survived', aggfunc='mean')
pivot_1

17. Pivot the table further to hold information in the following format:
     	    class 	First 	Second 	Third
sex 	age
female 	(0, 18] 	0.909091 	1.000000 	0.511628
(18, 80] 	0.972973 	0.900000 	0.423729
male 	(0, 18] 	0.800000 	0.600000 	0.215686
(18, 80] 	0.375000 	0.071429 	0.133663

# notice that the age, which is normally a number, is now a grouping (0-18, 18-80). We need to adjust our age column first, before we do another pivot
titanic['age'] = pd.cut(titanic['age'], [0, 18, 80])
titanic.head()

# assuming again, we want the average values of the survived column:
pivot_2 = titanic.pivot_table(index=['sex', 'age'], columns='class', values='survived', aggfunc='mean')
pivot_2

18.Pivot the table further to hold information in the following format:
fare 	survived
class 	First 	Second 	Third 	First 	Second 	Third
sex
female 	106.125798 	21.970121 	16.118810 	91.0 	70.0 	72.0
male 	67.226127 	19.741782 	12.661633 	45.0 	17.0 	47.0

pivot_3 = titanic.pivot_table(index='sex', columns='class', values=['fare', 'survived'], aggfunc={'fare': 'mean', 'survived': 'sum'})
pivot_3

!pip install mysql-connector-python

import pandas as pd
import mysql.connector

# Replace with your actual database credentials
mydb = mysql.connector.connect(
  host="sql5.freemysqlhosting.net",
  user="sql5757575",
  password="gCTYvUT6AN",
  database="sql5757575"
)

# Create a cursor object
mycursor = mydb.cursor()

sql_query = "SELECT * FROM countries"

# Read data into a pandas DataFrame
df = pd.read_sql(sql_query, mydb)
df.head()

query_21 = '''
SELECT street_address AS 'Street Address', city
FROM locations
WHERE city LIKE 'M%'
'''
df_21 = df = pd.read_sql(query_21, mydb)
df_21.head()

query_22 = '''
SELECT department_id, COUNT(*) AS employee_count
FROM employees
GROUP BY department_id
'''
df_22 = pd.read_sql(query_22, mydb)
df_22.head()

query_23 = '''
SELECT e.first_name, e.last_name, l.street_address, l.city, l.state_province, l.country_id
FROM employees e
JOIN departments d ON e.department_id = d.department_id
JOIN locations l ON d.location_id = l.location_id
'''
df_23 = pd.read_sql(query_23, mydb)
df_23.head()

mydb.close()



