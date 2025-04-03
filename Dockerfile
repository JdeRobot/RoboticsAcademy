# Use the official Ruby image with version 2.7.6 on a slim Debian base
FROM ruby:2.7.6-slim

# Install essential packages
RUN apt-get update -qq && apt-get install -y \
    build-essential \
    libpq-dev \
    zlib1g-dev \
    nodejs \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy Gemfile and Gemfile.lock for dependency installation
COPY Gemfile Gemfile.lock ./

# Install Ruby gems
RUN bundle install

# Expose ports
EXPOSE 4000
EXPOSE 35729

# Default command to start Jekyll, binding to all interfaces so it's accessible from the host.
CMD ["bundle", "exec", "jekyll", "serve"]