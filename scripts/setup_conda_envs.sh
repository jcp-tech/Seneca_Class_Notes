#!/bin/bash

echo "📦 Installing packages globally to system Python (pip3)..."
pip3 install --upgrade pip

# Packages for pip3
GENERAL_PACKAGES=(
  numpy pandas matplotlib scikit-learn seaborn scipy python-dotenv
  tensorflow opencv-python cmake dlib jupyter discord.py db-sqlite3
  mysql-connector-python statsmodels kagglehub ucimlrepo xgboost
  google-cloud-aiplatform transformers tpot xlrd
)

for pkg in "${GENERAL_PACKAGES[@]}"; do
  echo "➡️ Installing (pip3): $pkg"
  if pip3 install "$pkg"; then
    echo "✅ Installed: $pkg"
  else
    echo "❌ Failed: $pkg (skipped)"
  fi
done

# PyTorch packages with CUDA index
PYTORCH_PACKAGES=(torch torchvision torchaudio)

for pkg in "${PYTORCH_PACKAGES[@]}"; do
  echo "➡️ Installing (pip3 - cu118): $pkg"
  if pip3 install "$pkg" --index-url https://download.pytorch.org/whl/cu118; then
    echo "✅ Installed: $pkg"
  else
    echo "❌ Failed: $pkg (skipped)"
  fi
done

echo "✅ All global pip3 packages attempted."

# # --- Conda Base Environment ---
# echo "🔁 Activating conda base environment..."
# source "$(conda info --base)/etc/profile.d/conda.sh"
# conda activate base

# echo "📦 Installing packages in Conda base..."

# for pkg in "${GENERAL_PACKAGES[@]}"; do
#   echo "➡️ Installing (conda base): $pkg"
#   if pip install "$pkg"; then
#     echo "✅ Installed: $pkg"
#   else
#     echo "❌ Failed: $pkg (skipped)"
#   fi
# done

# for pkg in "${PYTORCH_PACKAGES[@]}"; do
#   echo "➡️ Installing (conda base - cu118): $pkg"
#   if pip install "$pkg" --index-url https://download.pytorch.org/whl/cu118; then
#     echo "✅ Installed: $pkg"
#   else
#     echo "❌ Failed: $pkg (skipped)"
#   fi
# done

# # --- OpenCV Environment ---
# echo "🔧 Creating OpenCV environment: 'ocv'"
# conda create -y -n ocv python=3.9
# conda activate ocv

# echo "➡️ Installing opencv-python==4.11.0.0 in 'ocv' env..."
# if pip install opencv-python==4.11.0.0; then
#   echo "✅ OpenCV installed successfully."
# else
#   echo "❌ OpenCV install failed."
# fi

# echo "🎉 All environments set up!"
