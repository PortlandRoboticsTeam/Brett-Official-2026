
# Portland Robotics 7195 - 2026 Competition Bot

Competition code for team 7195's robot.

## References

### Swerve Drive
Official Documentation: https://docs.yagsl.com/
GitHub: https://yet-another-software-suite.github.io/YAGSL/yagsl.json

### WCP 2026 "Big Dumper" Competitive Concept
https://wcproducts.com/pages/wcp-competitive-concepts

Video Demo: https://www.youtube.com/watch?v=wO9aJNpCE8Q
GitHub: https://github.com/wcpllc/2026CompetitiveConcept

## Control Guide
Slides: https://docs.google.com/presentation/d/1Df-ZnbTBpNsHdl3szxJ4QNOpd_V1Ua5YG6Antzs98B0/edit?slide=id.g3bec3249f21_0_50#slide=id.g3bec3249f21_0_50

# Can IDs: 

### Drive Control
 1. CTRE	Kraken		Forward Right Steering Motor
 2. CTRE	Kraken		Forward Right Drive Motor
 3. CTRE	CANcoder	Forward Right Steering Encoder
 4. CTRE	Kraken		Rear Right Steering Motor
 5. CTRE	Kraken		Rear Right Drive Motor
 6. CTRE	CANcoder	Rear Right Steering Encoder
 7. CTRE	Kraken		Rear Left Steering Motor
 8. CTRE	Kraken		Rear Left Drive Motor
 9. CTRE	CANcoder	Rear Left Steering Encoder
10. CTRE	Kraken		Forward Left Steering Motor
11. CTRE	Kraken		Forward Left Drive Motor
12. CTRE	CANcoder	Forward Left Steering Encoder

### Fuel Handling
20.	___		_______		Intake Pivot
21.	___		_______		Intake Rollers
22.	___		_______		Floor
23.	___		_______		Feeder
24.	___		_______		Left Shooter Flywheel
25.	___		_______		Middle Shooter Flywheel
26.	___		_______		Right Shooter Flywheel
28.	___		_______		Hanger

### Top-Level Hardware
60. REV		PDP			Central Power Distribution
61.	CTRE	CANdle		CANdle (Phoenix 5)
62.	CTRE	Pigeon 2	Central IMU