# State

There are two representations of drivetrain state:

- **Model100** represents measurements. Measurements never include acceleration, since it is not directly measurable.
- **Control100** represents control outputs, which _do_ contain acceleration, which can translate directly into motor voltages using the "kA" factor of the motor models.