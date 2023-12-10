function [T, Vm, Im, Vemf, RPM, D] = readData(filename)
    // Open the binary file
    fileID = mopen(filename, 'rb');

    // Check if the file is opened successfully
    if fileID == -1 then
        error('Error opening file.');
    end

    // Initialize arrays
    T = [];
    Vm = [];
    Im = [];
    Vemf = [];
    RPM = [];
    D = [];

    adc_max = 4095;
    uV = 3300000;
    Risense = 1500;
    Rm = 5;
    Rdson = 0.3;
    Apropri = 1500;
    isense_conversion_factor = uV * 1000 / adc_max / Risense / Apropri;
    frequency = 84000000;
    sample_rate = 6250;
    max_duty_cycle = frequency / sample_rate - 1;
    Vout = 5000; // milli volts
    Ke = 18.497 // RPM/mV
    
    pot_filtered = 0;
    alpha_pot = 0.05;
    isense_filtered = 0;
    alpha_isense = 0.05;

    // Read the file until the end
    while meof(fileID) == 0 do
        // Read a 32-bit unsigned integer
        tick = mget(1, 'ui', fileID);

        // Read a 16-bit unsigned integer
        isense = mget(1, 'us', fileID);
        isense_filtered = (1 - alpha_isense) * isense_filtered + alpha_isense * isense;

        // Read a 16-bit unsigned integer
        pot = mget(1, 'us', fileID);
        pot_filtered = (1 - alpha_pot) * pot_filtered + alpha_pot * pot;
        
        duty = pot_filtered / adc_max;
        
        // convert tick to time
        t = tick / frequency;

        // convert duty cycle into V
        v = Vout * duty;

        // convert isense value to motor current (mA)
        i = isense_filtered * isense_conversion_factor * duty;
        
        // calculate back emf
        vemf = v - i * (Rm + Rdson * 2);
        
        // calculate rpm
        rpm = Ke * vemf;

        // Append the values to the arrays
        T = [T t];
        Vm = [Vm v];
        Im = [Im i];
        Vemf = [Vemf vemf];
        RPM = [RPM rpm];
        D = [D duty];
    end

    mclose(fileID);

    T = T(100:$);
    Vm = Vm(100:$);
    Im = Im(100:$);
    Vemf = Vemf(100:$);
    RPM = RPM(100:$);    
    D = D(100:$);

    t1 = T(1);
    T = T - t1;
endfunction


[T, Vm, Im, Vemf, RPM, D] = readData('logs/data.bin');
