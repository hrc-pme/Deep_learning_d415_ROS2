#!/usr/bin/env bash
set -euo pipefail

TARGET_UID="${LOCAL_UID:-1000}"
TARGET_GID="${LOCAL_GID:-1000}"
USER_NAME="${NB_USER:-hrc}"

if [ "$(id -u)" -eq 0 ]; then
  # group
  if getent group "${TARGET_GID}" >/dev/null 2>&1; then
    GROUP_NAME="$(getent group "${TARGET_GID}" | cut -d: -f1)"
  else
    groupmod -g "${TARGET_GID}" "${USER_NAME}" 2>/dev/null || groupadd -g "${TARGET_GID}" "${USER_NAME}"
    GROUP_NAME="${USER_NAME}"
  fi

  # user
  CURRENT_UID="$(id -u "${USER_NAME}")"
  if [ "${CURRENT_UID}" != "${TARGET_UID}" ]; then
    usermod -u "${TARGET_UID}" -g "${TARGET_GID}" "${USER_NAME}" || true
  else
    usermod -g "${TARGET_GID}" "${USER_NAME}" || true
  fi

  HOME_DIR="$(getent passwd "${USER_NAME}" | cut -d: -f6)"
  chown -R "${TARGET_UID}:${TARGET_GID}" "${HOME_DIR}" /home/"${USER_NAME}" 2>/dev/null || true

  exec gosu "${USER_NAME}":"${TARGET_GID}" "$@"
fi

exec "$@"
