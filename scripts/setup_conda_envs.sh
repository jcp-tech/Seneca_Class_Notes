#!/bin/bash

echo "📦 Installing packages globally to system Python (pip3)..."
pip3 install --upgrade pip

pip3 install numpy pandas matplotlib scikit-learn seaborn scipy python-dotenv \
torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
tensorflow opencv-python cmake dlib jupyter discord.py db-sqlite3 \
mysql-connector-python statsmodels kagglehub ucimlrepo xgboost \
google-cloud-aiplatform transformers tpot xlrd

echo "✅ All packages installed globally via pip3."

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate base

echo "📦 Installing AI/ML/Data Science libraries in base environment..."
pip install --upgrade pip
pip install numpy pandas matplotlib scikit-learn seaborn scipy python-dotenv \
torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
tensorflow opencv-python cmake dlib jupyter discord.py db-sqlite3 \
mysql-connector-python statsmodels kagglehub ucimlrepo xgboost \
google-cloud-aiplatform transformers tpot xlrd

echo "🔧 Creating OpenCV environment: 'ocv'"
conda create -y -n ocv python=3.9
conda activate ocv
pip install opencv-python==4.11.0.0

echo "✅ All environments set up!"