# 🚀 GCP Docker + Cloud Run Deployment Guide

**Project:** Income Classification API  
**GCP Project ID:** `spendify-mapple-masala`

---

## Table of Contents

1. [Prerequisites](#-prerequisites)
  a. [Install Google Cloud SDK (gcloud)](#-a-install-google-cloud-sdk-gcloud)
  b. [Authenticate and Initialize GCP](#-b-authenticate-and-initialize-gcp)
2. [Project Setup & API Enablement](#-1-project-setup--api-enablement)
  a. [List All GCP Projects](#a-list-all-gcp-projects)
  b. [Set Active Project](#b-set-active-project)
  c. [Confirm Active Project](#c-confirm-active-project)
  d. [Enable Required GCP Services](#d-enable-required-gcp-services)
3. [Build & Deploy the API](#-2-build--deploy-the-api)
  a. [Delete Old Image (Optional)](#a-optional-delete-old-image)
  b. [Build & Push Docker Image](#b-build--push-docker-image)
  c. [Verify Image in Registry](#c-verify-image-in-registry)
  d. [Deploy to Cloud Run](#d-deploy-to-cloud-run)
4. [Testing the API](#-3-testing-the-api)
  a. [Swagger UI](#a-swagger-ui)
  b. [Test via Postman or curl](#b-test-via-postman-or-curl)
5. [Useful Links](#-useful-links)

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
gcloud run deploy income-api --image gcr.io/spendify-mapple-masala/income-api --platform managed --region us-central1 --allow-unauthenticated
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

## 🔗 Useful Links

* 🌐 [Cloud Run Dashboard](https://console.cloud.google.com/run?project=spendify-mapple-masala)
* 📦 [Container Registry Viewer](https://console.cloud.google.com/gcr/images/spendify-mapple-masala)
* 🛠 [Cloud Build History](https://console.cloud.google.com/cloud-build/builds?project=spendify-mapple-masala)