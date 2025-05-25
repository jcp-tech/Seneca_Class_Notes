#!/bin/bash

CONTAINER_NAME="ros-melodic"

if docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME$"; then
  echo "✅ Attaching to running container: $CONTAINER_NAME"
  docker exec -it "$CONTAINER_NAME" bash
else
  echo "❌ Container '$CONTAINER_NAME' is not running."
  echo "💡 Starting it now..."
  docker start "$CONTAINER_NAME"
  sleep 2
  docker exec -it "$CONTAINER_NAME" bash
fi
