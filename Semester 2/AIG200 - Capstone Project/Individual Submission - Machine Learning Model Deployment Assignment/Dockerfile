# Use a Python base image with enough support for Keras and numpy
FROM python:3.10-slim

# Avoid interactive prompts during dependencies install
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /app

# Copy all files
COPY . .

# Install OS-level dependencies (for numpy, pandas, etc.)
RUN apt-get update && \
    apt-get install -y --no-install-recommends gcc libglib2.0-0 libsm6 libxext6 libxrender-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Expose FastAPI port
EXPOSE 8080

# Run the FastAPI app with uvicorn
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8080"]