#ifndef TIMER_HPP
#define TIMER_HPP

class Timer {
private:
    long long int start_time;
    bool is_running;

    // Funzione privata: solo il cronometro sa come leggere l'orologio di sistema
    long long int current_time_ms(); 

public:
    Timer(); // Costruttore
    
    void start();
    void reset();
    
    bool has_elapsed(int target_ms); 
    long long int get_elapsed_time_ms(); 
};

#endif