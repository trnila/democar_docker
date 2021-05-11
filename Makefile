IMAGE=democarros

build:
	docker build -t $(IMAGE) .

recreate:
	docker rm -f /ros || true
	docker run \
		-v /tmp:/tmp \
		-v $(PWD):/ws/ \
		-d \
		--name ros -it $(IMAGE)

start: setup
	docker start ros

setup:
			sudo sysctl -w net.ipv4.ipfrag_time=3
			sudo sysctl -w net.ipv4.ipfrag_high_thresh=234217728
			sudo sysctl -w net.core.rmem_max=2147483647

enter:
	DISPLAY=:0 xhost +localhost
	docker exec -it ros /usr/local/bin/entry.sh
