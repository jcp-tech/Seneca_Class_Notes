# Run with `uvicorn app:app --reload` & Test on http://localhost:8000/docs
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import pandas as pd
import numpy as np
import joblib, os

script_dir = os.path.dirname(os.path.abspath(__file__))

# Initialize FastAPI app
app = FastAPI(
    title="Phishing Website Detection API",
    description="Send website feature inputs and get a prediction (0 = legitimate, 1 = phishing).",
    version="1.0.0"
)

# Define all feature names in order (based on your df)
FEATURES = [
    "having_ip_address", "url_length", "shortining_service", "having_at_symbol",
    "double_slash_redirecting", "prefix_suffix", "having_sub_domain", "sslfinal_state",
    "domain_registration_length", "favicon", "port", "https_token", "request_url",
    "url_of_anchor", "links_in_tags", "sfh", "submitting_to_email", "abnormal_url",
    "redirect", "on_mouseover", "rightclick", "popupwindow", "iframe", "age_of_domain",
    "dnsrecord", "web_traffic", "page_rank", "google_index", "links_pointing_to_page",
    "statistical_report"
]

# Load model pipeline
try:
    model = joblib.load(os.path.join(script_dir, "artifacts", "model.pkl"))
except Exception as e:
    raise RuntimeError(f"❌ Failed to load model: {e}")

# Define request schema
class PhishingInput(BaseModel):
    features: list[list[int]]

@app.post("/predict")
def predict(data: PhishingInput):
    try:
        input_array = np.array(data.features)
        if input_array.shape[1] != len(FEATURES):
            raise HTTPException(
                status_code=422,
                detail=f"Each row must have {len(FEATURES)} features"
            )

        # Wrap as DataFrame with column names
        input_df = pd.DataFrame(input_array, columns=FEATURES)

        predictions = model.predict(input_df)
        return {"predictions": predictions.tolist()}

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))