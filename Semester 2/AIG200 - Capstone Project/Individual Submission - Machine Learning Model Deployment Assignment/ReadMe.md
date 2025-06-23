# 🚀 GCP Docker + Cloud Run Deployment Guide

**Project:** Income Classification API
**GCP Project ID:** `spendify-mapple-masala`

---

## 📌 Prerequisites

Ensure the following tools are installed and you're authenticated:

### ✅ a. Install Google Cloud SDK (gcloud)

#### 🧱 Step A: Add the GCP CLI Package Source

```bash
sudo apt-get update && sudo apt-get install -y apt-transport-https ca-certificates gnupg
```

```bash
echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" | \
sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
```

```bash
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | \
sudo gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg
```

#### 🧱 Step B: Install the SDK

```bash
sudo apt-get update && sudo apt-get install -y google-cloud-sdk
```

---

### 🔐 b. Authenticate and Initialize GCP

```bash
gcloud auth login
gcloud init
```

> Ensure billing is enabled and the project is already set ✅

---

## 🌐 1. Project Setup & API Enablement

### a. List All GCP Projects

```bash
gcloud projects list
```

### b. Set Active Project

```bash
gcloud config set project spendify-mapple-masala
```

### c. Confirm Active Project

```bash
gcloud config get-value project
```

### d. Enable Required GCP Services

```bash
gcloud services enable run.googleapis.com containerregistry.googleapis.com cloudbuild.googleapis.com
```

### e. List Cloud Run Services

```bash
gcloud run services list
```

### f. Delete Cloud Run Service

```bash
gcloud run services delete income-api --region=us-central1
```

---

## 📄 2. Build & Deploy the API

### a. (Optional) Delete Old Image

```bash
gcloud container images delete gcr.io/spendify-mapple-masala/income-api --quiet
```

> ⚠️ Use only if you want to free space or force a clean rebuild.

---

### b. Build & Push Docker Image

Ensure you're in the project directory (contains `Dockerfile`, `app.py`, `requirements.txt`, `artifacts/`):

```bash
gcloud builds submit --tag gcr.io/spendify-mapple-masala/income-api
```

---

### c. Verify Image in Registry

```bash
gcloud container images list-tags gcr.io/spendify-mapple-masala/income-api
```

---

### d. Deploy to Cloud Run

```bash
gcloud run deploy income-api --image gcr.io/spendify-mapple-masala/income-api --platform managed --region us-central1 --allow-unauthenticated --memory 1Gi
```

> ✅ You’ll get a public URL like:
> `https://income-api-xxxxx.a.run.app`

---

## ✅ 3. Testing the API

### a. Swagger UI

```plaintext
https://income-api-xxxxx.a.run.app/docs
```

---

### b. Test via Postman or curl

```bash
curl -X POST https://income-api-xxxxx.a.run.app/predict \
  -H "Content-Type: application/json" \
  -d '{
  "inputs": [
  {
  "age": "Young",
  "workclass": "Private",
  "education_num": 13,
  "marital_status": "Never-married",
  "occupation": "Sales",
  "relationship": "Not-in-family",
  "race": "White",
  "sex": "Female",
  "hours_per_week": 40,
  "native_country": "United-States",
  "capital_profit": 1
  }
  ]
  }'
```

**Expected Response:**

```json
{
  "predictions": [0]
}
```

---

## 📝 4. Checking Logs

If you encounter errors or want to debug your Cloud Run service, use the Cloud Logging API.
**Note:** The correct command is `gcloud logging read` (not `gcloud logs read`).

### a. View Recent Logs for Your Service

```bash
gcloud logging read "resource.type=cloud_run_revision AND resource.labels.service_name=income-api" --project=spendify-mapple-masala --limit=50 --freshness=1h --format="value(textPayload)"
```

### b. (Optional) Show Full JSON Output

For more detailed debugging, use:

```bash
gcloud logging read "resource.type=cloud_run_revision AND resource.labels.service_name=income-api" --project=spendify-mapple-masala --limit=50 --freshness=1h --format=json
```

This will display the actual error messages your container printed during startup, helping you identify issues such as missing files, crashes, or incorrect paths.

---

## 🔗 Useful Links

* 🌐 [Cloud Run Dashboard](https://console.cloud.google.com/run?project=spendify-mapple-masala)
* 📦 [Container Registry Viewer](https://console.cloud.google.com/gcr/images/spendify-mapple-masala)
* 🛠 [Cloud Build History](https://console.cloud.google.com/cloud-build/builds?project=spendify-mapple-masala)