# Use the official Ruby image with version 2.7.6 on a slim Debian base
FROM ruby:2.7.8

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
COPY ../Gemfile ./
COPY ../Gemfile.lock ./

# Install Ruby gems
RUN bundle install

# Expose ports
EXPOSE 4000
EXPOSE 35729

# Default command to start Jekyll server
CMD ["bundle", "exec", "jekyll", "serve"]