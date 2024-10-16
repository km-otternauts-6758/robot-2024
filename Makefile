deploy:
	python -m robotpy deploy

sync:
	py -3 -m robotpy sync

info: 
	py -m robotpy deploy-info

list:
	py -m robotpy list

sim:
	python -m robotpy sim
delete:
	py -m robotpy undeploy
commit:
	git commit -asm "Swerve Update"
push:
	git push origin main