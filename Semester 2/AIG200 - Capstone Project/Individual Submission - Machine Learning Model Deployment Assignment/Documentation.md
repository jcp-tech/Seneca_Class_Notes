# 🚀 GCP Docker + Cloud Run Deployment Guide

**Project: Income Classification API**
**GCP Project ID:** `spendify-mapple-masala`

---

## 📌 Prerequisites

Ensure the following tools are installed and you're authenticated:

* [Google Cloud SDK (gcloud)](https://cloud.google.com/sdk/docs/install)
* Docker (for local builds)
* Authenticated via:

```bash
gcloud auth login
```

* Initialized using:

```bash
gcloud init
```

* Billing account and project are already set ✅

---

## 🌐 1. Project Setup & API Enablement

### 🔍 List All GCP Projects

```bash
gcloud projects list
```

### 🔄 Set Active Project

```bash
gcloud config set project spendify-mapple-masala
```

### ✅ Confirm Active Project

```bash
gcloud config get-value project
```

### 🔓 Enable Required GCP Services

```bash
gcloud services enable run.googleapis.com containerregistry.googleapis.com cloudbuild.googleapis.com
```

---

## 📄 2. Build & Deploy the API

### 🧹 Step 1: (Optional) Delete Old Image

```bash
gcloud container images delete gcr.io/spendify-mapple-masala/income-api --quiet
```

> ⚠️ Use only if you want to free space or force a clean rebuild.

---

### 🔁 Step 2: Build & Push Docker Image

Ensure you're in the project directory (contains `Dockerfile`, `app.py`, `requirements.txt`, `artifacts/`):

```bash
gcloud builds submit --tag gcr.io/spendify-mapple-masala/income-api
```

> This:
>
> * Packages your FastAPI app
> * Builds it using Cloud Build
> * Pushes it to Container Registry

---

### 🔎 Step 3: Verify Image in Registry

```bash
gcloud container images list-tags gcr.io/spendify-mapple-masala/income-api
```

---

### ☁️ Step 4: Deploy to Cloud Run

```bash
gcloud run deploy income-api --image gcr.io/spendify-mapple-masala/income-api --platform managed --region us-central1 --allow-unauthenticated
```

> You’ll receive a live public endpoint like:
> `https://income-api-xxxxx.a.run.app`

---

## ✅ 3. Testing the API

### 📄 Open the Swagger UI:

```plaintext
https://income-api-xxxxx.a.run.app/docs
```

### 🧪 Test via Postman or `curl`:

```http
POST https://income-api-xxxxx.a.run.app/predict
Content-Type: application/json
```

**Example Request Body:**

```json
{
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
}
```

**Response:**

```json
{
  "predictions": [0]
}
```

---

## 🔗 Useful Links

* 🌐 [Cloud Run Dashboard](https://console.cloud.google.com/run?project=spendify-mapple-masala)
* 📦 [Container Registry Viewer](https://console.cloud.google.com/gcr/images/spendify-mapple-masala)
* 🛠 [Cloud Build History](https://console.cloud.google.com/cloud-build/builds?project=spendify-mapple-masala)
