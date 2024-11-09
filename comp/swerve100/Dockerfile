# Use an official OpenJDK runtime as a parent image
FROM eclipse-temurin:17-jdk

# Install necessary tools
RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /app

# Install Gradle
ENV GRADLE_VERSION=8.4
RUN curl -L https://services.gradle.org/distributions/gradle-${GRADLE_VERSION}-bin.zip -o gradle.zip \
    && unzip gradle.zip \
    && rm gradle.zip \
    && mv gradle-${GRADLE_VERSION} /opt/gradle \
    && ln -s /opt/gradle/bin/gradle /usr/bin/gradle

# Set Gradle environment variables
ENV GRADLE_HOME=/opt/gradle
ENV PATH=$PATH:$GRADLE_HOME/bin

# Copy just the build.gradle and gradle files
COPY build.gradle settings.gradle gradle.properties ./
COPY gradle ./gradle
# COPY gradlew ./

# Make the gradlew script executable
# RUN chmod +x ./gradlew

# Download dependencies
RUN --mount=type=cache,target=/root/.gradle gradle --build-cache dependencies

# Now copy the rest of your code
COPY . .

# Build the project
# RUN --mount=type=cache,target=/root/.gradle gradle --build-cache build --offline

# Expose the JMX port
# EXPOSE 1099

# Run the Java application
# CMD ["gradle", "run"]
