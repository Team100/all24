# Use the same WPILib container as the GitHub Action
FROM wpilib/roborio-cross-ubuntu:2024-22.04

# Set working directory
WORKDIR /team100/comp/swerve100

# Copy only wrapper files first
COPY comp/swerve100/gradle ./gradle
COPY comp/swerve100/gradlew comp/swerve100/gradlew.bat ./

# Make gradlew executable (same as in GitHub Action)
RUN chmod +x gradlew

# Copy build configuration
COPY comp/swerve100/*.gradle comp/swerve100/gradle.properties ./

# Download dependencies using wrapper
RUN ./gradlew

# Lib doesn't change often; copy before the other code
WORKDIR /team100
COPY lib ./lib

# Now copy the rest of your code
WORKDIR /team100/comp/swerve100
COPY comp/swerve100/src ./src

# Build and test (same command as in GitHub Action)
RUN ./gradlew build