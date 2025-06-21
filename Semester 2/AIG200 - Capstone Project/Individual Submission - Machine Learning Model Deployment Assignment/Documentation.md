Here's your clean, copy-paste ready **GCP Docker + Cloud Run Deployment Documentation**:

---

## 📄 **Deployment & Redeployment Guide – Phishing Detection API**

### 🧹 1. Delete Existing Container Image from Google Container Registry

```bash
gcloud container images delete gcr.io/algebraic-pier-396215/phishing-api --quiet
```

> `--quiet` skips confirmation prompts.
> This removes the existing container image to free up space or force a fresh build.

---

### 🔁 2. Rebuild Docker Image and Push to GCP

From your project folder (with `Dockerfile`, `app.py`, `requirements.txt`):

```bash
gcloud builds submit --tag gcr.io/algebraic-pier-396215/phishing-api
```

> This will:
>
> * Package your app
> * Build it on Google Cloud Build
> * Push it to Container Registry at `gcr.io/...`

---

### ☁️ 3. Deploy to Cloud Run (Public API)

```bash
gcloud run deploy phishing-api --image gcr.io/algebraic-pier-396215/phishing-api --platform managed --region us-central1 --allow-unauthenticated
```

> This will:
>
> * Deploy your container as a live REST API
> * Provide a public URL like:
>   `https://phishing-api-xxxxx.a.run.app`

---

### ✅ Test After Deployment

* Go to `https://phishing-api-xxxxx.a.run.app/docs` in your browser
* Or use Postman to send a `POST` request to `/predict`

---

Let me know if you want a `README.md` version or markdown PDF export for your report!
