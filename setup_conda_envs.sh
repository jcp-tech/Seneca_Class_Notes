#!/bin/bash

echo "🔧 Creating AI environment: 'ai' with Python 3.12"
conda create -y -n ai python=3.12
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate ai

echo "📦 Installing packages in 'ai' environment..."
pip install numpy pandas matplotlib scikit-learn seaborn scipy python-dotenv \
torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
tensorflow opencv-python cmake dlib jupyter discord.py db-sqlite3 \
mysql-connector-python statsmodels kagglehub ucimlrepo xgboost \
google-cloud-aiplatform transformers tpot xlrd || echo "⚠️ Some packages may have failed."

echo "🔧 Creating OpenCV environment: 'ocv'"
conda create -y -n ocv python=3.9
conda activate ocv
pip install opencv-python==4.11.0.0

echo "✅ All environments set up!"