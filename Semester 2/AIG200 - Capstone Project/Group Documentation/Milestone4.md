# Milestone 4 (Week 12): Near-Complete System & Deployment

---

## AI Capstone: Milestone 4 Check-in (Week 12)

**Focus:** Demonstrate a near-complete, end-to-end system, ideally deployed to a staging/test environment. Emphasis is on system functionality, cloud deployment, thorough testing, and readiness for final polishing/documentation.

---

### 1. End-to-End System Demonstration

* **Live demonstration of the system performing its core function:**

  * **API Project:** API calls (via Postman, curl, or Discord bot) to deployed endpoints, returning results from real data.
  * **Dashboard Project:** (Planned/partially available) Deployed dashboard will show insights from processed/model data.
  * **Pipeline Project:** (Partially in CI) Model/classification pipeline is deployed and registers new sessions/data.
  * **Integrated Demo:** All components work together: Discord Bot → Flask API → Document AI (OCR) → ADK Classification Pipeline → Firebase DB.
* **Workflow & User Experience:**

  * User uploads receipts in Discord; receives classified results and summaries.
  * API endpoints are accessible for direct integration/testing (links below).
  * Data flows from upload to structured analysis and storage.

---

### 2. Deployment Status

* **Deployment URLs / Access:**
  <!-- Need to Disable after Semester. -->
  * **ADK Classification API/UI:** [https://receipt-classifier-32377413295.us-central1.run.app/](https://receipt-classifier-32377413295.us-central1.run.app/)
  * **Flask API (Dashboard backend):** [https://spendify-api-32377413295.us-central1.run.app/](https://spendify-api-32377413295.us-central1.run.app/)
  * **Discord Bot:** [Invite link](https://discord.com/oauth2/authorize?client_id=1375145987106144297&permissions=8&integration_type=0&scope=bot)
* **Final Deployment Configuration:**

  * All components are containerized and run on Google Cloud (Cloud Run/VM).
  * Env and secret configuration is managed per deployment guide (see deploy-\*.md in repo).
  * Firebase is the core persistent storage. APIs use secure cloud endpoints.
* **Deployment Challenges:**

  * GCP ADK pipeline had steep learning curve due to minimal docs/examples.
  * Integration, permissions, and service account setup required trial/error.
  * Addressed cloud auth, API versioning, and data hand-off edge cases.
* **Remaining Steps/Blockers:**

  * Dashboard: Frontend for user summaries is in progress for the final milestone.
  * Regression model integration for spend prediction is ongoing.
  * Real-world OCR edge cases being actively tested and patched.

---

### 3. Testing & Validation

* **Testing Performed:**

  * Manual end-to-end runs: Full cycle from Discord upload to output and dashboard query.
  * Unit tests: Core API, classification, and storage logic.
  * Model validation: Holdout receipts used to check classification accuracy.
  * User acceptance: Real users tested Discord bot, provided feedback.
* **Key Findings:**

  * Ollama-based LLM models replaced with more accurate GCP ADK agent pipeline.
  * Classification results are robust for most receipt formats.
  * API endpoints are stable, return consistent and valid JSON.
  * Data storage and retrieval in Firebase is reliable.
* **Bugs Found and Fixed:**

  * JSON serialization for nested/complex structures fixed for Firestore.
  * Error handling improved for bad receipts and cloud service errors.
  * API response structure made more consistent and robust.
* **Model Validation Post-Deployment:**

  * Outputs compared against ground-truth labels for accuracy.
  * All pipeline agents validated for schema adherence and performance.

---

### 4. Documentation & Final Steps

* **Documentation Outline/Draft:**

  * Repo includes README, deployment guides, code comments, and system diagrams (process.png).
  * Final report (per guidelines) and presentation slides are being drafted.
* **Plan for Next \~2 Weeks:**

  * Polish dashboard frontend for release.
  * Integrate regression model into pipeline and dashboard.
  * Finalize and submit report & presentation.
  * Final bug fixes, testing, and polish.
* **Remaining Risks:**

  * Cloud cost overruns.
  * User data quality and edge-case handling (OCR/classification).
  * Dashboard/analytics frontend may require rapid iteration for best UX.
