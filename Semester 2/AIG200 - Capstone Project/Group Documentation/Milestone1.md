## Milestone 1 (Week 3): Project Definition & Planning

### 1. Project Charter

**Initial Idea:** Build an AI-based system that integrates WhatsApp/Discord bots for image upload, uses GCP Vertex AI for OCR processing, performs classification and regression modeling for financial insights, and delivers outputs via a web dashboard.

**Problem Statement:** Manual receipt management is inefficient and lacks intelligence. Users need automated tools to track expenses, categorize spending, and gain personalized insights.

**Project Goal:** Develop a chatbot-integrated, cloud-hosted platform that extracts structured data from receipts and provides categorized expense tracking along with predictive financial advice.

**In-Scope:**

* Scanning and processing printed receipts
* Cloud-hosted API for pipeline management
* Google Vision/Vertex Document AI for extraction
* NLP-based categorization and visualization
* Classification model to categorize receipt items
* Regression model for user-specific financial predictions
* AI agent (Langchain/Ollama) for spending insights
* Discord bot integration
* Support for PDFs, scanned/unscanned images

**Out-of-Scope:**

* Handwritten receipt recognition
* Mobile applications
* Full user authentication system
* Web dashboard for summaries and insights
* Webhook-based automation
* Custom API Integrations for Customer who want to Integrate into their Own CRM Systems
* WhatsApp bot integration

### 2. Data Acquisition & Initial Exploration

**Data Sources:**

* Public receipt datasets
* User uploads via bot/web interface

**Initial Findings:**

* Predominantly English
* Format inconsistencies
* Requires cleaning and schema standardization

**Cleaning Plan:**

* Remove promotional noise and irrelevant lines
* Normalize structure and unify schemas
* Tag store, items, prices, tax
* Extract unique Purchase IDs
* Label items by category for classification
* Aggregate user-wise weekly spend for regression

### 3. Technical Approach

**Models/Algorithms:**

* OCR: Google Vision API → Vertex Document AI
* Categorization: spaCy/BERT (TBD)
* Classification: Model to categorize items (e.g., Groceries, Electronics) — *current plan is to utilize an LLM (e.g., Langchain/Ollama) to provide this classification as needed*
* Regression: Model to predict weekly spend
* LLM Agent: Langchain/Ollama for advice generation

**Tech Stack:**

* Backend: Flask/Django (TBD), Docker
* Hosting: GCP/AWS/Hostinger (TBD)
* Storage: MySQL/Firebase (TBD)
* Bots: Currently only Discord
* Visualization: Streamlit/Dash (TBD)

**Architecture Highlights:**

* CI/CD + cronjobs for deployment
* OCR → classification → semantic tagging
* Aggregated & categorized data → regression model
* On-demand and scheduled AI responses
* Jenkins, Git, Kubernetes for orchestration

### 4. Project Plan & Team Roles

**Image Upload & Bot Integration:**

* Aliyyah: Bot weekly summary implementation
* Jonathan: Discord bot + pipeline trigger

**Data Ingestion & Routing:**

* Jonathan: API design + data format exploration

**OCR & Processing:**

* Aadil: OCR accuracy tests
* Jonathan: Google Vision API + Vertex AI setup

**Data Cleaning & Preprocessing:**

* Aadil: Normalize text + define schema
* Aliyyah: Remove redundant data

**NLP Categorization:**

* Aadil: NLP model testing and categorization logic

**Classification Model:**

* TBD: Label data, train/test classification, iterate on results

**Regression Model:**

* TBD: Aggregate weekly data, train/test predictions, tune output

**AI Agent Layer:**

* Jonathan: Langchain/Ollama integration
* Aliyyah: LLM documentation + testing
* Aadil: LLM prototyping + support

**Frontend Development:**

* Aliyyah: Streamlit mockups
* Aadil: Dash exploration
* Jonathan: Web login/dashboard

**Project Management:**

* All: Use Notion + weekly syncs + deployment decisions

### 5. Risks and Mitigation

* **OCR Errors:** Use custom-trained Vertex AI models
* **Bot Downtime:** Use SDKs + modular bot design
* **Model Overfitting:** Diverse data + fine-tuning
* **Cloud Costs:** Use GCP free tier, fallback to alternatives
* **Automation Errors:** Redundancy for webhooks/cronjobs

### 6. Professionalism & Engagement

* **Preparedness:** Weekly logs, clear tasks, assigned roles
* **Engagement:** Weekly syncs, tech exploration, early bot prototyping

### 7. Process Flow (End-to-End)

1. **Image Capture & Upload:**

   * Users upload via Discord/WhatsApp

2. **API Routing:**

   * Bot/API collects image and metadata
   * Routes to GCP bucket

3. **Document Parsing:**

   * GCP Vertex AI extracts structured fields (store, items, tax, ID)

4. **Classification Model:**

   * Items are categorized into types
   * Totals per category calculated

5. **Regression Model:**

   * Predicts next week’s spend using aggregated user data

6. **AI Agent Response:**

   * Langchain/Ollama generate advice based on predictions

7. **Storage:**

   * Final structured data saved to MySQL/Firebase

8. **Output Delivery:**

   * Summary to users (bot) or JSON to business clients

9. **Automation & CI/CD:**

   * Cronjobs/webhooks handle scheduling
   * Deployment via Jenkins + Docker + Kubernetes

### Mermaid Diagram

![Mermaid Diagram](https://kroki.io/mermaid/svg/eNpVU01zm0AMvfdX6AfYBzvJJYfOYDA2jjtxQ1IfdnxYsAwU2GX2o4mb9L9Xu0um1BdG0tOTnt760snXsubKwHPyBegXsReNCl6GTvKzhicssRkMZD2v8ATz-VdYvWdisAbimguB3R_ftnKlj6TRpVTnD4gXLEdxBiMhOmTw2pgaPG8mLhI4VQ5K9oM5TZqPNTc6GgbXPUmn1liFcMSCmg2qCy_RQzwmXvidEubGzCHtuG5BKkh-clFJymylNngGKSDupD3fQ3TMZ7CJDzMHc9VGVKjCIoknW7OgPoh2GggOK1u2OC689riUkYgStQ76HOgHKoNvkMjS9igMRNkp7Jn6jg1bvxnFS7dRwg2_h9xIEueuMoPMYK9n8CwN7-h7sIqc0VRNwtSN59iyKIOocuxz2JNKAjXCiXnsOt7zgN16bMZiOohuLleIucFKqgaJmUa7j7Mht33PVfMbx7Hjuplv373nkpag2nXA4PNu6rMb-s-1hwV7QvJKkChurPZVxLa7jlOupwlFTPZzcQ2gAg5S0XgiWX6SpLbriEnZ0vkf7vXfdnsWjkfiXa1wp5rDt2v-fe9Y00ahy4092haV4kMNCf56HDTJlZ32FW_8gpFn7ec78Kkl2zRmEt-wHYq2EXqSu2UPtkAlkA46Sd-xWNGL28lC-yuTwlrKdoTQHyPstA9vdzENltPgZhrcToO7v0SgF40)

<!-- ![Mermaid Diagram](https://kroki.io/mermaid/svg/eNp1VMtu2zAQvOcr9tSTgkLqC_ChgPWw6jRt0jppDkIQUNJaYi2JAh9p3K_vkpRlB4V9sA1qZnc4O9ptJ_5ULZMa7tILoM-y2GgcIVzAumcNQsJGbSQCG2q4HzvB6ke4vPwMscdFC1jeruELPe740DjYT2E0_X909WKHTjz63QJSUZkeBw3LNdwyqWZg4oCpB74nINOMumtshOR_meZigHJvacuG-J6UOlLmSR8WsNFCWtVWxsb0PZN7D8wccOWBHxdwY_RoNKTY8Wc8gFYOlHvQJ7qZ0aL3nW3B9bCVTGlpKuvI44XjKFM2ko2tt-vJW-SeODfD4l6hBOOOFUiskI8anjmDlKtKyDqAh5ZptRzHAISEsWN6K2TvFeFQv-5CZj9N_s5N4rCIhbZkO4pKdB1WWgG3ggLoUbOavAzcHUYp-nEyz3GjwpZDBVpALkTT0cg7YerZythUO9Rn5Bym-TSNcq6bhMUvlBpfXg2culeoFE7ijjKSqMhetGRWt6LGCHzYigC4xl4FoNkLfQnNOn-LdXpGz-u8zOXTsDjkBozVCddsaCj3fLC23XQd69lRTRoVd6xRUPlq-wDIQJwUqMlII4mv8LyWycC5ahYWG3c1Ow26H3zbb35c2_4rLrGkWkcFWVTcShwZoZWPMVAoiKnaUjBZW1ZPTrJmfn_-6-8jPpdc-ShOGXxGCsqw5XLKN9VTp--LY0RFbN3yEzvQrjY33-EP1y3UlCzeqXNJnV-euWAeFsn6bZJOM8i5Dmw8digDuMJhxwcy96spUQ5IkTwqyaPiActWiJ1y5ldSDL9FqZwpqmqxNh3WFBO1O5Hj95l7p5fhyTKKQ_8TneydxJ8l0claSf1ZGp1skMyfZdHJwliF8IbMcie5XyH2JI8u_gFFrZoO) -->