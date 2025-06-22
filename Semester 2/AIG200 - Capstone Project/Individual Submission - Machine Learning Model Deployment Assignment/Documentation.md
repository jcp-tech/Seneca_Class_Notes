# 🚀 GCP Docker + Cloud Run Deployment Guide

**Project: Phishing Detection API**
**GCP Project ID:** `spendify-mapple-masala`

---

## 📌 Prerequisites

Ensure you have the following tools installed and authenticated:

* [Google Cloud SDK (gcloud)](https://cloud.google.com/sdk/docs/install)
* Docker (if building locally)
* Logged into `gcloud` with:

```bash
gcloud auth login
```

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

### 🔓 Enable Required Services

```bash
gcloud services enable run.googleapis.com containerregistry.googleapis.com
```

---

## 📄 2. Deployment & Redeployment Guide

### 🧹 Step 1: Delete Old Container Image (Optional)

```bash
gcloud container images delete gcr.io/spendify-mapple-masala/phishing-api --quiet
```

> Use this only if you want to remove old builds before pushing a new one.
> `--quiet` skips confirmation.

---

### 🔁 Step 2: Build & Push Docker Image

From your app directory (must include `Dockerfile`, `app.py`, `requirements.txt`):

```bash
gcloud builds submit --tag gcr.io/spendify-mapple-masala/phishing-api
```

> This:
>
> * Packages your app
> * Builds it via Google Cloud Build
> * Pushes it to: `gcr.io/spendify-mapple-masala/phishing-api`

---

### 🔎 Step 3: Verify Image Exists in Container Registry

```bash
gcloud container images list-tags gcr.io/spendify-mapple-masala/phishing-api
```

> Ensures the image is correctly pushed and available for deployment.

---

### ☁️ Step 4: Deploy to Cloud Run

```bash
gcloud run deploy phishing-api --image gcr.io/spendify-mapple-masala/phishing-api --platform managed --region us-central1 --allow-unauthenticated
```

> This:
>
> * Deploys the container
> * Makes it publicly accessible
> * Returns a live URL like:
>   `https://phishing-api-xxxxx.a.run.app`

---

## ✅ 3. Testing the API

Once deployed:

* Open API Docs (if using FastAPI or similar):
  `https://phishing-api-xxxxx.a.run.app/docs`

* Test via Postman:

  ```http
  POST https://phishing-api-xxxxx.a.run.app/predict
  Content-Type: application/json
  ```

  Example Body:

  ```json
  {
    "url": "http://example.com"
  }
  ```

---

## 🔗 Useful Links

* 🌐 [Cloud Run Dashboard](https://console.cloud.google.com/run?inv=1&invt=Ab0wYg&project=spendify-mapple-masala)
* 📦 [Container Registry Viewer](https://console.cloud.google.com/gcr/images/spendify-mapple-masala)
