"""Base models that work on pydantic v1 and v2.

The container's system python ships pydantic 1.10 (python3-pydantic on
noble) while the workspace uv.lock pins 2.x - this package runs on either.
Code here uses the v2 method names (model_dump, model_copy,
model_validate_json); on v1 they map to dict / copy / parse_raw.
"""
import pydantic
from pydantic import BaseModel

PYDANTIC_V2 = int(pydantic.VERSION.split('.')[0]) >= 2

if PYDANTIC_V2:
    from pydantic import ConfigDict

    class Model(BaseModel):
        model_config = ConfigDict(extra='ignore')

    class FrozenModel(BaseModel):
        """Immutable and hashable."""
        model_config = ConfigDict(frozen=True, extra='ignore')
else:
    class _V2Api:
        def model_dump(self, **kwargs):
            return self.dict(**kwargs)

        def model_copy(self, update=None, deep=False):
            return self.copy(update=update, deep=deep)

        @classmethod
        def model_validate_json(cls, data):
            return cls.parse_raw(data)

    class Model(BaseModel, _V2Api):
        class Config:
            extra = 'ignore'

    class FrozenModel(BaseModel, _V2Api):
        """Immutable and hashable."""
        class Config:
            frozen = True
            extra = 'ignore'
