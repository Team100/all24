# Use an official OpenJDK runtime as a parent image
FROM eclipse-temurin:17-jdk

# Install necessary tools
RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /app

# Copy just the build.gradle and gradle files
COPY build.gradle settings.gradle gradle.properties ./
COPY gradle ./gradle
COPY gradlew ./

# Make the gradlew script executable
RUN chmod +x ./gradlew

# Download dependencies
RUN --mount=type=cache,target=/root/.gradle ./gradlew --build-cache dependencies

# Now copy the rest of your code
COPY . .

# Build the project
RUN --mount=type=cache,target=/root/.gradle ./gradlew --build-cache build --offline

# Expose the JMX port
EXPOSE 1099

# Run the Java application
CMD ["./gradlew", "run"]
