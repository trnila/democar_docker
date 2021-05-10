#IMAGE=192.168.10.1:2000/democarros
IMAGE=democarros

build:
	docker build -t $(IMAGE) .
#	docker push $(IMAGE)

create:
	docker run --privileged \
		-v /tmp:/tmp \
		-v $(PWD)/ws:/ws \
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
