IMAGE=trnila/democar

build:
	docker build -t $(IMAGE) .
	docker push $(IMAGE)

recreate:
	docker pull $(IMAGE)
	docker rm -f /ros || true
	touch .bash_history
	docker run \
		--network host \
		-v /tmp:/tmp \
		-e DISPLAY=$(DISPLAY) \
		-v $(PWD):/ws/ \
		-v $(PWD)/.bash_history:/root/.bash_history \
		-v $(HOME)/.Xauthority:/root/.Xauthority \
		-d \
		--name ros -it $(IMAGE)

start: setup
	docker start ros

enter:
	xhost +localhost
	docker exec -it ros /usr/local/bin/entry.sh
