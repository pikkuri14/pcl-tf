#include <iostream>
#include <vector>


class Fibonacci
{
public:
    Fibonacci(int count) : count_(count), prime_(0) {}
    
    std::vector<int> generate() {
        std::vector<int> sequence;
        if (count_ <= 0) return sequence;
        if (count_ >= 1) sequence.push_back(0);
        if (count_ >= 2) sequence.push_back(1);

        for (int i = 2; i < count_; ++i) {
            int next_seq = sequence[i - 1] + sequence[i - 2];
            sequence.push_back(next_seq);

            primeCounter(next_seq);
        }

        return sequence;
    }

    int primeGetter(){
        return prime_;
    }

    
private:

    bool isPrime(int number) {

        if (number <= 1) {
            return false; 
        }

        for (int i = 2; i * i <= number; ++i) { // Check up to the square root of the number
            if (number % i == 0) {
                return false;
            }
        }

        return true; // No divisors found, it is prime
    }

    void primeCounter(int num){
        if(isPrime(num)) prime_++;
    }

    int count_;
    int prime_; 
    
};


int main(int argc, char const *argv[])
{
    Fibonacci fib(20);
    std::vector<int> result = fib.generate();
    int prime = fib.primeGetter();
    std::cout << "sequence: ";

    for (int num : result) {
        std::cout << num << " ";
    }
    std::cout << std::endl;
    std::cout << "number of prime: " << prime << std::endl;
    return 0;
}
