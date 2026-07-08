from __future__ import annotations

from typing import Any


def get_vector_feature_labels(feature_spec: dict[str, Any], vector_length: int) -> list[str]:
    names = feature_spec.get("names") or []
    if len(names) == vector_length:
        return list(names)
    if len(names) == 1:
        return [f"{names[0]}[{idx}]" for idx in range(vector_length)]
    return [f"dim_{idx}" for idx in range(vector_length)]
