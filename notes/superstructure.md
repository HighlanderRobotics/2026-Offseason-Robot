# 2026 Offseason Superstructure

## States
- IDLE (nothing running, maybe have flywheel stay spun up somewhat)
- INTAKE (just intake, maybe indexer needed to move balls into robot)
- SCORE (Score into goal, index and shoot)
- FEED (Shoot at feed zone, index and shoot)
- SCORE_FLOW (Intake and shoot into hub at the same time)
- FEED_FLOW (Intake and shoot at feed zone at same time)
- SPIN_UP_SCORE (wait for flywheel to spin to velocity before indexing)
- SPIN_UP_FEED (wait for flywheel to spin up before indexing)

## Transitions
IDLE goes to INTAKE on an intake request, and INTAKE returns to IDLE when the request ends
IDLE goes to SPIN_UP_(FEED/SCORE) when a the score/feed request is active, SPIN_UP_(FEED/SCORE) goes to (FEED/SCORE) when the flywheel is at the correct velocity
FEED and SCORE return to IDLE when the request ends
IDLE to flow instead if flow is requested