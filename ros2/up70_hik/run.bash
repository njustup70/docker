#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g)}"

cmd="${1:-up}"

case "${cmd}" in
  up)
    docker compose -f "${COMPOSE_FILE}" up -d
    ;;
  down)
    docker compose -f "${COMPOSE_FILE}" down
    ;;
  logs)
    docker compose -f "${COMPOSE_FILE}" logs -f --tail=200
    ;;
  shell)
    docker compose -f "${COMPOSE_FILE}" exec up70_hik bash
    ;;
  *)
    echo "Usage: ${0##*/} {up|down|logs|shell}"
    exit 2
    ;;
esac
