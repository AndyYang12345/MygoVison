#ifndef GENETIC_ALGORITHM_HPP
#define GENETIC_ALGORITHM_HPP

#include <algorithm>
#include <functional>
#include <random>
#include <vector>
#include <cmath>

struct Genome {
    float p = 0.8f;
    float i = 0.0f;
    float d = 0.15f;

    float p_min = 0.0f;
    float p_max = 5.0f;
    float i_min = 0.0f;
    float i_max = 1.0f;
    float d_min = 0.0f;
    float d_max = 2.0f;

    void clamp() {
        p = std::max(p_min, std::min(p, p_max));
        i = std::max(i_min, std::min(i, i_max));
        d = std::max(d_min, std::min(d, d_max));
    }

    void mutate(std::mt19937& rng, float rate = 0.2f, float sigma = 0.1f) {
        std::normal_distribution<float> noise(0.0f, sigma);
        std::uniform_real_distribution<float> prob(0.0f, 1.0f);
        if (prob(rng) < rate) p += noise(rng);
        if (prob(rng) < rate) i += noise(rng);
        if (prob(rng) < rate) d += noise(rng);
        clamp();
    }

    static Genome crossover(const Genome& a, const Genome& b, std::mt19937& rng) {
        std::uniform_real_distribution<float> mix(0.0f, 1.0f);
        float t = mix(rng);
        Genome out = a;
        out.p = a.p * t + b.p * (1.0f - t);
        out.i = a.i * t + b.i * (1.0f - t);
        out.d = a.d * t + b.d * (1.0f - t);
        out.p_min = a.p_min; out.p_max = a.p_max;
        out.i_min = a.i_min; out.i_max = a.i_max;
        out.d_min = a.d_min; out.d_max = a.d_max;
        out.clamp();
        return out;
    }
};

struct Individual {
    int id = -1;
    int generation = 0;
    Genome genome;
    float fitness = 0.0f;
    bool evaluated = false;
};

class Population {
public:
    using FitnessFunc = std::function<float(const Genome&)>;

    Population(int size, unsigned int seed = std::random_device{}())
        : _rng(seed), _size(size) {
        _individuals.reserve(size);
    }

    void set_fitness_function(FitnessFunc func) {
        _fitness = std::move(func);
    }

    struct TrackingFitnessConfig {
        int targets_per_individual = 5;
        float dt = 0.01f;
        float max_time = 2.0f;
        float settle_threshold = 0.01f; // rad
        float settle_hold = 0.2f;       // sec
        float max_rate_rad = 1.5f;      // rad/s
        float integral_limit = 1.0f;
        float weight_time = 1.0f;
        float weight_error = 1.0f;
        float weight_smooth = 0.05f;
        float target_pitch_min = -0.35f;
        float target_pitch_max = 0.35f;
        float target_yaw_min = -0.6f;
        float target_yaw_max = 0.6f;
        unsigned int seed = 12345;
    };

    void set_tracking_fitness(const TrackingFitnessConfig& cfg) {
        _tracking_config = cfg;
        _tracking_rng.seed(cfg.seed);
        _fitness = [this](const Genome& genome) {
            return evaluate_tracking_fitness(genome);
        };
    }

    void set_mutation(float rate, float sigma) {
        _mutation_rate = rate;
        _mutation_sigma = sigma;
    }

    void set_elitism(int elite_count) {
        _elite_count = std::max(0, elite_count);
    }

    void initialize_random(const Genome& base) {
        std::uniform_real_distribution<float> up(0.0f, 1.0f);
        _individuals.clear();
        for (int i = 0; i < _size; ++i) {
            Genome g = base;
            g.p = g.p_min + (g.p_max - g.p_min) * up(_rng);
            g.i = g.i_min + (g.i_max - g.i_min) * up(_rng);
            g.d = g.d_min + (g.d_max - g.d_min) * up(_rng);
            g.clamp();
            Individual ind;
            ind.id = i;
            ind.generation = _generation;
            ind.genome = g;
            ind.fitness = 0.0f;
            ind.evaluated = false;
            _individuals.push_back(ind);
        }
    }

    void evaluate_all() {
        if (!_fitness) {
            return;
        }
        for (auto& ind : _individuals) {
            if (!ind.evaluated) {
                ind.fitness = _fitness(ind.genome);
                ind.evaluated = true;
            }
        }
        sort_by_fitness();
    }

    const Individual& best() const {
        return _individuals.front();
    }

    const std::vector<Individual>& individuals() const {
        return _individuals;
    }

    int generation() const {
        return _generation;
    }

    void evolve_next() {
        if (_individuals.empty()) {
            return;
        }
        sort_by_fitness();

        std::vector<Individual> next;
        next.reserve(_size);

        int elite = std::min(_elite_count, static_cast<int>(_individuals.size()));
        for (int i = 0; i < elite; ++i) {
            Individual copy = _individuals[i];
            copy.generation = _generation + 1;
            copy.evaluated = false;
            next.push_back(copy);
        }

        while (static_cast<int>(next.size()) < _size) {
            const Individual& parent_a = tournament_select(3);
            const Individual& parent_b = tournament_select(3);
            Genome child = Genome::crossover(parent_a.genome, parent_b.genome, _rng);
            child.mutate(_rng, _mutation_rate, _mutation_sigma);
            Individual offspring;
            offspring.id = static_cast<int>(next.size());
            offspring.generation = _generation + 1;
            offspring.genome = child;
            offspring.fitness = 0.0f;
            offspring.evaluated = false;
            next.push_back(offspring);
        }

        _individuals = std::move(next);
        _generation += 1;
    }

private:
    std::mt19937 _rng;
    std::mt19937 _tracking_rng{12345};
    int _size = 0;
    int _generation = 0;
    float _mutation_rate = 0.2f;
    float _mutation_sigma = 0.1f;
    int _elite_count = 2;
    FitnessFunc _fitness;
    TrackingFitnessConfig _tracking_config;
    std::vector<Individual> _individuals;

    void sort_by_fitness() {
        std::sort(_individuals.begin(), _individuals.end(),
                  [](const Individual& a, const Individual& b) {
                      return a.fitness > b.fitness;
                  });
    }

    const Individual& tournament_select(int k) {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(_individuals.size()) - 1);
        int best_index = pick(_rng);
        for (int i = 1; i < k; ++i) {
            int idx = pick(_rng);
            if (_individuals[idx].fitness > _individuals[best_index].fitness) {
                best_index = idx;
            }
        }
        return _individuals[best_index];
    }

    struct PidState {
        float integral = 0.0f;
        float prev_error = 0.0f;
        bool has_prev = false;
    };

    static float wrap_angle(float angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    float step_pid(float target, float current, float dt, const Genome& g, PidState& state) const {
        float error = wrap_angle(target - current);
        state.integral += error * dt;
        state.integral = std::max(-_tracking_config.integral_limit,
                                  std::min(state.integral, _tracking_config.integral_limit));

        float derivative = 0.0f;
        if (state.has_prev && dt > 1e-6f) {
            derivative = (error - state.prev_error) / dt;
        }
        state.prev_error = error;
        state.has_prev = true;

        float rate = g.p * error + g.i * state.integral + g.d * derivative;
        rate = std::max(-_tracking_config.max_rate_rad, std::min(rate, _tracking_config.max_rate_rad));
        return current + rate * dt;
    }

    float evaluate_tracking_fitness(const Genome& genome) {
        std::uniform_real_distribution<float> pitch_dist(_tracking_config.target_pitch_min,
                                                         _tracking_config.target_pitch_max);
        std::uniform_real_distribution<float> yaw_dist(_tracking_config.target_yaw_min,
                                                       _tracking_config.target_yaw_max);

        float total_cost = 0.0f;
        for (int t = 0; t < _tracking_config.targets_per_individual; ++t) {
            float target_pitch = pitch_dist(_tracking_rng);
            float target_yaw = yaw_dist(_tracking_rng);

            float current_pitch = 0.0f;
            float current_yaw = 0.0f;
            float time_elapsed = 0.0f;
            float settle_timer = 0.0f;
            float error_integral = 0.0f;
            float smooth_penalty = 0.0f;
            float prev_rate_pitch = 0.0f;
            float prev_rate_yaw = 0.0f;

            PidState pitch_state;
            PidState yaw_state;

            while (time_elapsed < _tracking_config.max_time) {
                float prev_pitch = current_pitch;
                float prev_yaw = current_yaw;

                current_pitch = step_pid(target_pitch, current_pitch, _tracking_config.dt, genome, pitch_state);
                current_yaw = step_pid(target_yaw, current_yaw, _tracking_config.dt, genome, yaw_state);

                float rate_pitch = (current_pitch - prev_pitch) / _tracking_config.dt;
                float rate_yaw = (current_yaw - prev_yaw) / _tracking_config.dt;

                float dp = std::abs(wrap_angle(target_pitch - current_pitch));
                float dy = std::abs(wrap_angle(target_yaw - current_yaw));
                error_integral += (dp + dy) * _tracking_config.dt;

                smooth_penalty += (std::abs(rate_pitch - prev_rate_pitch) +
                                   std::abs(rate_yaw - prev_rate_yaw)) * _tracking_config.dt;
                prev_rate_pitch = rate_pitch;
                prev_rate_yaw = rate_yaw;

                if (dp < _tracking_config.settle_threshold && dy < _tracking_config.settle_threshold) {
                    settle_timer += _tracking_config.dt;
                    if (settle_timer >= _tracking_config.settle_hold) {
                        time_elapsed += _tracking_config.dt;
                        break;
                    }
                } else {
                    settle_timer = 0.0f;
                }

                time_elapsed += _tracking_config.dt;
            }

            float cost = _tracking_config.weight_time * time_elapsed +
                         _tracking_config.weight_error * error_integral +
                         _tracking_config.weight_smooth * smooth_penalty;
            total_cost += cost;
        }

        return -total_cost;
    }
};

#endif // GENETIC_ALGORITHM_HPP