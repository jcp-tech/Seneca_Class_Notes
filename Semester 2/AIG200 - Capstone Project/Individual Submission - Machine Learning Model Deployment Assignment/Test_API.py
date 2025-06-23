# import random, requests, json, os
# from dotenv import load_dotenv
# load_dotenv()  # Load environment variables from .env file

data_options = {
    "age": ["Middle-aged", "Young", "Old"],
    "workclass": [
        "Private", "Self-emp-not-inc", "Local-gov", "State-gov",
        "Self-emp-inc", "Federal-gov", "Without-pay"
    ],
    "education-num": [9, 10, 13, 14, 11, 7, 12, 6, 4, 15, 5, 8, 16, 3, 2, 1],
    "marital-status": [
        "Married-civ-spouse", "Never-married", "Divorced", "Separated",
        "Widowed", "Married-spouse-absent", "Married-AF-spouse"
    ],
    "occupation": [
        "Prof-specialty", "Craft-repair", "Exec-managerial", "Adm-clerical", "Sales",
        "Other-service", "Machine-op-inspct", "Transport-moving", "Handlers-cleaners",
        "Farming-fishing", "Tech-support", "Protective-serv", "Priv-house-serv", "Armed-Forces"
    ],
    "relationship": [
        "Husband", "Not-in-family", "Own-child", "Unmarried", "Wife", "Other-relative"
    ],
    "race": ["White", "Black", "Asian-Pac-Islander", "Amer-Indian-Eskimo", "Other"],
    "sex": ["Male", "Female"],
    "hours-per-week": [
        40, 50, 45, 60, 35, 20, 30, 55, 25, 48, 38, 15, 70, 65, 32, 10, 24, 42, 44,
        36, 16, 43, 37, 12, 52, 80, 8, 56, 46, 99, 28, 18, 75, 72, 47, 6, 84, 22, 54,
        5, 39, 33, 41, 14, 90, 26, 27, 58, 17, 34, 4, 49, 3, 21, 53, 23, 7, 62, 13, 57,
        9, 66, 2, 64, 19, 51, 85, 68, 98, 11, 63, 78, 1, 77, 29, 96, 31, 67, 59, 76,
        91, 81, 97, 89, 73, 88, 95, 61, 86, 87, 94, 82, 92, 74
    ],
    "native-country": [
        "United-States", "Mexico", "Philippines", "Germany", "Puerto-Rico", "Canada",
        "El-Salvador", "India", "Cuba", "England", "Jamaica", "South", "Italy", "China",
        "Dominican-Republic", "Vietnam", "Guatemala", "Japan", "Poland", "Columbia",
        "Taiwan", "Haiti", "Iran", "Portugal", "Nicaragua", "Peru", "Greece", "Ecuador",
        "France", "Ireland", "Hong", "Trinadad&Tobago", "Cambodia", "Thailand", "Laos",
        "Yugoslavia", "Outlying-US(Guam-USVI-etc)", "Hungary", "Honduras", "Scotland", "Holand-Netherlands"
    ],
    "capital-profit": [1, 0]
}

# # Pick a random sample
# random_sample = {
#     feature: random.choice(options)
#     for feature, options in data_options.items()
# }

# # Convert to API’s expected keys (aliases)
# api_input = {
#     "age": random_sample["age"],
#     "workclass": random_sample["workclass"],
#     "education-num": random_sample["education-num"],
#     "marital-status": random_sample["marital-status"],
#     "occupation": random_sample["occupation"],
#     "relationship": random_sample["relationship"],
#     "race": random_sample["race"],
#     "sex": random_sample["sex"],
#     "hours-per-week": random_sample["hours-per-week"],
#     "native-country": random_sample["native-country"],
#     "capital-profit": random_sample["capital-profit"]
# }

# payload = {
#     "inputs": [api_input]
# }

# # --- CONFIG --- #
# API_URL = "https://income-api-32377413295.us-central1.run.app/predict"
# API_KEY = os.environ.get("INCOME_API_KEY")


# # --- Send Request --- #
# headers = {
#     "Content-Type": "application/json",
#     "x-api-key": API_KEY
# }

# print("Request payload:")
# print(json.dumps(payload, indent=2))

# try:
#     response = requests.post(API_URL, headers=headers, json=payload, timeout=20)
#     print("\nStatus Code:", response.status_code)
#     try:
#         print("Response JSON:", response.json())
#     except Exception:
#         print("Raw response text:")
#         print(response.text)
# except Exception as e:
#     print("Request failed:", str(e))
