
DOCKER_USERNAME ?= jderobot
BASE_NAME ?= robotics-applications
APP_NAME ?= robotics-academy
BUILD_TAG ?= test
# VERSION ?= 3.2.14

all: help

help: ## Print help info
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""

# Docker building
build:  ## Build RADI
	@docker build --no-cache=true -t $(DOCKER_USERNAME)/$(APP_NAME):test .

build-base:  ## Build RADI base
	@docker build -f Dockerfile.base -t $(DOCKER_USERNAME)/$(BASE_NAME):base .

build-all: build-base build  ## Build RADI and base

# Docker run
run:
	@echo Not implemented

# Docker pull
pull:  ## Pull latest RADI
	@docker pull $(DOCKER_USERNAME)/$(APP_NAME):latest

prune:  ## Prune all images
	@docker system prune

clean:  ## Remove RADI image
	@docker rmi -f $(DOCKER_USERNAME)/$(APP_NAME)


ifeq ($(VERSION),)
$(info VERSION not set! Run it again with VERSION="X.Y.Z")

else
# Docker release
release: build publish  ## Make a release by building and publishing the `{version}` and `latest` tagged containers to dockerhub

# Docker publish
publish: publish-latest publish-version  ## Publish the `{version}` and `latest` tagged containers to dockerhub

publish-latest: tag-latest ## Publish the `latest` taged container to dockerhub
	@echo 'publish latest to $(DOCKER_USERNAME)'
	@docker push $(DOCKER_USERNAME)/$(APP_NAME):latest

publish-version: tag-version ## Publish the `{version}` taged container to dockerhub
	@echo 'publish $(VERSION) to $(DOCKER_REPO)'
	@docker push $(DOCKER_USERNAME)/$(APP_NAME):$(VERSION)

# Docker tagging
tag: tag-latest tag-version ## Generate container tags for the `{version}` ans `latest` tags

tag-latest: ## Generate container `{version}` tag
	@echo 'create tag latest'
	@docker tag jderobot/robotics-academy:test $(DOCKER_USERNAME)/$(APP_NAME):latest

tag-version: ## Generate container `latest` tag
	@echo 'create tag $(VERSION)'
	@docker tag jderobot/robotics-academy:test $(DOCKER_USERNAME)/$(APP_NAME):$(VERSION)

version: ## Output the current version
	@echo $(VERSION)

endif
