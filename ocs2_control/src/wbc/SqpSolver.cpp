#include "ocs2_control/wbc/SqpSolver.h"

#include <qpOASES.hpp>

namespace switched_model {

vector_t SqpSolver::solve_sqp(const Task &weightedTasks, const Task &constraints) {
    // Number of decision variables
    const size_t nDecisionVariables = weightedTasks.A.cols();

    // Number of constraints
    const size_t nConstraints = constraints.A.rows() + constraints.D.rows();

    // Setup constraint lbA <= A*x <= ubA
    matrix_qp A = (matrix_qp(nConstraints, nDecisionVariables) << constraints.A, constraints.D).finished();
    vector_t lbA =
        (vector_t(nConstraints) << constraints.b, -qpOASES::INFTY * vector_t::Ones(constraints.f.size())).finished();
    vector_t ubA = (vector_t(nConstraints) << constraints.b, constraints.f).finished();

    // x.T @ H @ x + x.T @ g <---> (Ax - b).T @ (Ax - b)
    matrix_t weightedAT = weightedTasks.A.transpose();
    matrix_qp H = weightedAT * weightedTasks.A;
    vector_t g = -weightedAT * weightedTasks.b;

    // Setup QPOASES problem
    qpOASES::QProblem qp_problem(nDecisionVariables, nConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qp_problem.setOptions(options);
    int nWsr = 20;
    qp_problem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);

    // Solve QP problem
    vector_t solution(nDecisionVariables);
    qp_problem.getPrimalSolution(solution.data());
    return solution;
}

}  // namespace switched_model