# Run with `uvicorn app:app --reload` & test at http://localhost:8000/docs
print("The Code is running.")

from fastapi import FastAPI, HTTPException, Depends, Header, status
from pydantic import BaseModel, Field
from dotenv import load_dotenv
from datetime import datetime
import tensorflow as tf
from typing import List
import pandas as pd
import numpy as np
import joblib
import os

load_dotenv()  # Load environment variables from .env file

# Set script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
artifacts_dir = os.path.join(script_dir, "artifacts")
API_KEY = os.environ.get("INCOME_API_KEY")

def verify_api_key(x_api_key: str = Header(...)):
    if x_api_key != API_KEY:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing API key"
        )

# --- Load Preprocessor and Model ---
try:
    preprocessor = joblib.load(os.path.join(artifacts_dir, "preprocessing_pipeline.pkl"))
    model = tf.keras.models.load_model(os.path.join(artifacts_dir, "model.h5"))
except Exception as e:
    print(f"Failed to load model or preprocessor: {e}")
    raise RuntimeError(f"Failed to load model or preprocessor: {e}")

# --- FastAPI App ---
app = FastAPI(
    title="Income Classification API",
    description="Predict whether income >50K or <=50K based on demographic data.",
    version="2.0.0"
)

# --- Feature Schema ---
class IncomeInput(BaseModel):
    age: str
    workclass: str
    education_num: int = Field(..., alias="education-num")
    marital_status: str = Field(..., alias="marital-status")
    occupation: str
    relationship: str
    race: str
    sex: str
    hours_per_week: int = Field(..., alias="hours-per-week")
    native_country: str = Field(..., alias="native-country")
    capital_profit: int = Field(..., alias="capital-profit")
    class Config:
        allow_population_by_field_name = True

class InputBatch(BaseModel):
    inputs: List[IncomeInput]
    class Config:
        allow_population_by_alias = True

# --- Predict Endpoint ---
@app.post("/predict")
def predict(
    data: InputBatch,
    api_key: str = Depends(verify_api_key)
):
    try:
        # 🔒 Expiration check
        if datetime.now() > datetime(2025, 8, 27):
            raise HTTPException(
                status_code=403,
                detail="This API is no longer active after August 27, 2025."
            )
        
        # Convert list of Pydantic models to DataFrame
        input_data = pd.DataFrame([item.dict(by_alias=True) for item in data.inputs])

        # Preprocess
        X_processed = preprocessor.transform(input_data)

        # Predict using Keras model (assumes sigmoid output for binary classification)
        raw_preds = model.predict(X_processed)
        preds = [int(p > 0.5) for p in raw_preds.flatten()]

        return {"predictions": preds}
    
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
