graphics_toolkit ("gnuplot")



transisionsCW = csvread("hallStatesCW_faulhaber.csv");
transisionsCCW = csvread("hallStatesCCW_faulhaber.csv");
configs = csvread("CCConfigs.csv");


m = min(configs(:,2:end)(:));
configs(:,2:end) = configs(:,2:end) - m;
m = max(configs(:,2:end)(:));
configs(:,2:end) = configs(:,2:end) ./ m;

plot(
	configs(:,1),configs(:,2), "r",
	configs(:,1),configs(:,3), "g",
	configs(:,1),configs(:,4), "b",
	configs(:,1),configs(:,5), "r",
	configs(:,1),configs(:,6)+1, "r",
	configs(:,1),configs(:,7)+1, "g",
	configs(:,1),configs(:,8)+1, "b",
	configs(:,1),configs(:,9)+1, "r",
	transisionsCW(:,1),transisionsCW(:,2), "r",
	transisionsCW(:,1),transisionsCW(:,3), "g",
	transisionsCW(:,1),transisionsCW(:,4), "b",
	transisionsCCW(:,1),transisionsCCW(:,2)+1, "r",
	transisionsCCW(:,1),transisionsCCW(:,3)+1, "g",
	transisionsCCW(:,1),transisionsCCW(:,4)+1, "b"
)
pause();

transisionsCW = csvread("hallStatesCW.csv");
transisionsCCW = csvread("hallStatesCCW.csv");
configs = csvread("CCConfigs.csv");


m = min(configs(:,2:end)(:));
configs(:,2:end) = configs(:,2:end) - m;
m = max(configs(:,2:end)(:));
configs(:,2:end) = configs(:,2:end) ./ m;

plot(
	configs(:,1),configs(:,2), "r",
	configs(:,1),configs(:,3), "g",
	configs(:,1),configs(:,4), "b",
	configs(:,1),configs(:,5), "r",
	configs(:,1),configs(:,6)+1, "r",
	configs(:,1),configs(:,7)+1, "g",
	configs(:,1),configs(:,8)+1, "b",
	configs(:,1),configs(:,9)+1, "r",
	transisionsCW(:,1),transisionsCW(:,2), "r",
	transisionsCW(:,1),transisionsCW(:,3), "g",
	transisionsCW(:,1),transisionsCW(:,4), "b",
	transisionsCCW(:,1),transisionsCCW(:,2)+1, "r",
	transisionsCCW(:,1),transisionsCCW(:,3)+1, "g",
	transisionsCCW(:,1),transisionsCCW(:,4)+1, "b"
)
pause();
