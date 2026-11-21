"""Normalize Unreal Python optional-out trace returns, without guessing visibility."""
def trace_blocked(result):
    if result is None:return False
    if isinstance(result,bool):return result
    if isinstance(result,tuple) and result and isinstance(result[0],bool):return result[0]
    return True  # Native HitResult object denotes a collision.
