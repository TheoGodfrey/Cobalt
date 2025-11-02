# Use an official Python runtime as a parent image
# Using a slim image to keep the final size down
FROM python:3.10-slim

# Set environment variables for Python and Poetry
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV POETRY_NO_INTERACTION 1
ENV POETRY_VIRTUALENVS_CREATE false

# Install system dependencies
# These are required by opencv-python
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install poetry
RUN pip install poetry

# Set the main working directory
WORKDIR /app

# Copy the dependency definition file
# This is done first to leverage Docker layer caching
COPY pyproject.toml ./

# Install project dependencies
RUN poetry install

# Copy the entire project source code into the container
# This assumes the Docker build is run from the project root
COPY . .

# Set the final working directory to where main.py is located
# This ensures that main.py can find its config folder ("config/mission_config.yaml")
# and that imports from 'core', 'strategies', etc., work correctly.
WORKDIR /app/v_0.2/scout_drone

# Command to run the application
CMD ["python", "main.py"]