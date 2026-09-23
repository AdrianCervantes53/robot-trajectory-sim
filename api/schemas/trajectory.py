from pydantic import BaseModel, Field


class SaveTrajectoryRequest(BaseModel):
    """Save recorded trajectory by name."""
    name: str = Field(..., description="Trajectory name identifier.")


class LoadTrajectoryRequest(BaseModel):
    """Load saved trajectory by name."""
    name: str = Field(..., description="Trajectory name identifier.")
