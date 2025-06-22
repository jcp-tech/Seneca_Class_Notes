data_options = {
  "having_IP_Address": [
    1,
    -1
  ],
  "URL_Length": [
    -1,
    1,
    0
  ],
  "Shortining_Service": [
    1,
    -1
  ],
  "having_At_Symbol": [
    1,
    -1
  ],
  "double_slash_redirecting": [
    1,
    -1
  ],
  "Prefix_Suffix": [
    -1,
    1
  ],
  "having_Sub_Domain": [
    1,
    0,
    -1
  ],
  "SSLfinal_State": [
    1,
    -1,
    0
  ],
  "Domain_registeration_length": [
    -1,
    1
  ],
  "Favicon": [
    1,
    -1
  ],
  "port": [
    1,
    -1
  ],
  "HTTPS_token": [
    1,
    -1
  ],
  "Request_URL": [
    1,
    -1
  ],
  "URL_of_Anchor": [
    0,
    -1,
    1
  ],
  "Links_in_tags": [
    0,
    -1,
    1
  ],
  "SFH": [
    -1,
    1,
    0
  ],
  "Submitting_to_email": [
    1,
    -1
  ],
  "Abnormal_URL": [
    1,
    -1
  ],
  "Redirect": [
    0,
    1
  ],
  "on_mouseover": [
    1,
    -1
  ],
  "RightClick": [
    1,
    -1
  ],
  "popUpWidnow": [
    1,
    -1
  ],
  "Iframe": [
    1,
    -1
  ],
  "age_of_domain": [
    1,
    -1
  ],
  "DNSRecord": [
    1,
    -1
  ],
  "web_traffic": [
    1,
    -1,
    0
  ],
  "Page_Rank": [
    -1,
    1
  ],
  "Google_Index": [
    1,
    -1
  ],
  "Links_pointing_to_page": [
    0,
    1,
    -1
  ],
  "Statistical_report": [
    1,
    -1
  ],
  "Result": [
    1,
    -1
  ]
}

import random
import requests
import json

random_sample = {
    feature: random.choice(options)
    for feature, options in data_options.items()
    if feature != "Result"
}

random_features = list(random_sample.values())

# Your Cloud Run API endpoint
API_URL = "https://phishing-api-32377413295.us-central1.run.app/predict"

# Build payload
payload = {
    "features": [random_features]
}

print("Payload to send:", json.dumps(random_sample, indent=2))

# Make the POST request
response = requests.post(API_URL, json=payload)

# Output response
print("Status Code:", response.status_code)
print("Response JSON:", response.json())