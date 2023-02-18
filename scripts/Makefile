
DOCKER_USERNAME ?= jderobot
BASE_NAME ?= robotics-applications
APP_NAME ?= robotics-academy
BUILD_TAG ?= test


all: help

help: ## Print help info
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""


# Docker building
build:  ## Build RADI
	@docker build --no-cache=true -t $(DOCKER_USERNAME)/$(APP_NAME):$(BUILD_TAG) .

build-base:  ## Build RADI base
	@docker build -f Dockerfile.base -t $(DOCKER_USERNAME)/$(BASE_NAME):base .

build-all: build-base build  ## Build RADI and base


# Docker run test image
run-test:  ## Run last built RADI
	@docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:$(BUILD_TAG)

run:  ## Run RADI
	@echo See run instructions at https://jderobot.github.io/RoboticsAcademy/exercises/


# Docker pull
pull:  ## Pull latest RADI
	@docker pull $(DOCKER_USERNAME)/$(APP_NAME):latest

prune:  ## Prune all images
	@docker system prune

clean:  ## Remove RADI image
	@docker rmi -f $(DOCKER_USERNAME)/$(APP_NAME)


# Docker release
release: build publish  ## Build and publish the `{version}` and `latest` tagged containers to dockerhub


# Docker publish
publish: publish-version publish-latest  ## Publish the `{version}` and `latest` tagged containers to dockerhub

publish-version: tag-version ## Publish the `{version}` taged container to dockerhub
	@docker push $(DOCKER_USERNAME)/$(APP_NAME):$(VERSION)

publish-latest: tag-latest ## Publish the `latest` taged container to dockerhub
	@docker push $(DOCKER_USERNAME)/$(APP_NAME):latest


# Docker tagging
tag: tag-version tag-latest ## Generate container tags for the `{version}` ans `latest` tags

tag-version: version ## Generate container `latest` tag
	@docker tag jderobot/robotics-academy:$(BUILD_TAG) $(DOCKER_USERNAME)/$(APP_NAME):$(VERSION)

tag-latest: ## Generate container `{version}` tag
	@docker tag jderobot/robotics-academy:$(BUILD_TAG) $(DOCKER_USERNAME)/$(APP_NAME):latest


# Version
version: ## Output the current version
	@if [ -z "${VERSION}" ] ; then echo "VERSION must be set" ; false ; fi
	@echo VERSION=$(VERSION)
