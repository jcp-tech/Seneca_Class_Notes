import numpy as np
import pandas as pd

import os
BASE_URL = r"C:\Users\JonathanChackoPattas\OneDrive - Maritime Support Solutions\Desktop\Class Notes\Seneca\AIG150 - Data Preparation and Governance\Week 2\Lab"

customer = pd.read_csv(os.path.join(BASE_URL, "customer.csv")).filter(["CUS_CODE", "CUS_FNAME", "CUS_LNAME"])
customer

invoice = pd.read_csv(os.path.join(BASE_URL, "invoice.csv")).filter(["INV_NUMBER", "CUS_CODE", "INV_DATE"])
invoice

invoice['CUS_CODE'].value_counts()

customer = pd.concat([customer, pd.DataFrame({'CUS_CODE': [10020], 'CUS_FNAME': ['James'], 'CUS_LNAME': ['Smith']})], ignore_index = True)
customer.drop_duplicates(inplace=True)
customer

customerInvoice = pd.merge(customer, invoice, on='CUS_CODE', how='right')

pd.merge(customer, invoice, on='CUS_CODE', how='left')

pd.merge(customer, invoice, on='CUS_CODE', how='inner')

customerInvoice.to_csv(os.path.join(BASE_URL, "customerInvoice.csv"), index=False)

def github_to_raw_url(url):
    url = url.replace("https://github.com/", "https://raw.githubusercontent.com/")
    return url

population = pd.read_csv(github_to_raw_url(r"https://github.com/jakevdp/data-USstates/master/state-population.csv"))
area = pd.read_csv(github_to_raw_url(r"https://github.com/jakevdp/data-USstates/master/state-areas.csv"))
abbreviation = pd.read_csv(github_to_raw_url(r"https://github.com/jakevdp/data-USstates/master/state-abbrevs.csv"))

abbreviation # print(abbreviation)

area # print(area)

population # print(population)

merge_df = pd.merge(population, abbreviation, how='left', left_on='state/region', right_on='abbreviation') # state_abbrev_map = dict(zip(abbreviation['abbreviation'], abbreviation['state']))
merge_df.drop(columns=['abbreviation'], inplace=True)
merge_df

merge_df['state'].fillna(merge_df['state/region'], inplace=True)
merge_df

if 'PR' not in merge_df['state/region'].unique():
    puerto_rico_data = population[population['state/region'] == 'PR']
    puerto_rico_data['state'] = 'Puerto Rico'
    merge_df = pd.concat([merge_df, puerto_rico_data], ignore_index=True)
    merge_df['state'] = merge_df['state'].fillna(merge_df['state/region'])
# merge_df.drop(columns=['state/region'], inplace=True)
merge_df

final_state_df = pd.merge(merge_df, area, how='outer', on='state')
final_state_df

titanic = pd.read_csv(os.path.join(BASE_URL, "titanic.csv"))
titanic

titanic_pivot_v1 = titanic.pivot_table(index='sex', columns='class', values='survived', aggfunc='mean')
titanic_pivot_v1

titanic_df = titanic.copy()
titanic_df['age'] = titanic_df['age'].apply(lambda age: '(0, 18]' if age <= 18 else '(18, 80]')
titanic_pivot_v2 = titanic_df.pivot_table(index=['sex', 'age'], columns='class', values='survived', aggfunc='mean')
titanic_pivot_v2

titanic_pivot_v3 = titanic.pivot_table(index='sex', columns='class', values=['fare', 'survived'], aggfunc={'fare': 'mean', 'survived': 'sum'})
titanic_pivot_v3

import mysql.connector as msql

db = msql.connect(
    host="sql5.freemysqlhosting.net",
    user="sql5757575",
    password="gCTYvUT6AN",
    database="sql5757575"
)

# cursor = db.cursor()

countries_df = pd.read_sql("SELECT * FROM `countries`", con=db)
countries_df

# pd.read_sql("""
#     SELECT 
#         street_address AS 'Street Address', city
#     FROM 
#         `locations`
#     WHERE 
#         city LIKE 'M%'
# """, con=db)

locations_df = pd.read_sql("SELECT * FROM `locations`", con=db)
locations_df_q21 = locations_df.copy()
locations_df_q21 = locations_df_q21.filter(['street_address', 'city']).rename(columns={'street_address': 'Street Address'})
locations_df_q21 = locations_df_q21[locations_df_q21['city'].str.startswith('M')]
locations_df_q21

# pd.read_sql("""
#     SELECT 
#         department_id, COUNT(employee_id) AS total_employees
#     FROM 
#         `employees`
#     GROUP BY 
#         department_id
# """, con=db).dropna()

employees_df = pd.read_sql("SELECT * FROM `employees`", con=db)
employees_df_q22 = employees_df.copy()
employees_df_q22 = employees_df_q22.groupby('department_id').size().reset_index(name='total_employees').dropna()
employees_df_q22

# pd.read_sql("""
#     SELECT 
#         CONCAT(e.first_name, ' ', e.last_name) AS name,
#         CONCAT(l.street_address, ', ', l.city, ', ', l.state_province, ', ', l.postal_code, ', ', l.country_id) AS address
#     FROM 
#         employees e
#     LEFT JOIN 
#         departments d ON e.department_id = d.department_id
#     LEFT JOIN 
#         locations l ON d.location_id = l.location_id
#     ORDER BY 
#         name;
# """, con=db)

department_df = pd.read_sql("SELECT * FROM `departments`", con=db)
employees_df_q23 = employees_df.copy() 
employees_df_q23['department_id'] = employees_df_q23['department_id'].fillna(
    employees_df_q23['manager_id'].map(
        employees_df_q23.set_index('manager_id')['department_id'].dropna().to_dict()
    )
)
employees_df_q23 = employees_df_q23.merge(department_df, on='department_id', how='right')
employees_df_q23 = employees_df_q23[['first_name', 'last_name', 'location_id']]
employees_df_q23 = employees_df_q23.merge(locations_df, left_on='location_id', right_on='location_id', how='left')
employees_df_q23['name'] = employees_df_q23['first_name'] + ' ' + employees_df_q23['last_name']
employees_df_q23['address'] = (employees_df_q23['street_address'] + ', ' + 
                 employees_df_q23['city'] + ', ' + 
                 employees_df_q23['state_province'] + ', ' + 
                 employees_df_q23['postal_code'] + ', ' + 
                 employees_df_q23['country_id'])
employees_df_q23 = employees_df_q23.filter(['name', 'address']).dropna()
employees_df_q23.sort_values(by='name', inplace=True)
employees_df_q23

# cursor.close()
db.close()

