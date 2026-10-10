#include <cmath>
#include <iostream>

#include "nav2_amcl/pf/pf_pdf.hpp"

pf_pdf_gaussian_t * makePdf()
{
  pf_vector_t mean = pf_vector_zero();
  pf_matrix_t covariance = pf_matrix_zero();
  covariance.m[0][0] = 0.01;
  covariance.m[1][1] = 0.01;
  covariance.m[2][2] = 0.01;
  return pf_pdf_gaussian_alloc(mean, covariance);
}

bool equal(const pf_vector_t & a, const pf_vector_t & b)
{
  return a.v[0] == b.v[0] && a.v[1] == b.v[1] && a.v[2] == b.v[2];
}

pf_vector_t draw(long seed)
{
  pf_pdf_set_seed(seed);
  auto * pdf = makePdf();
  pf_vector_t sample = pf_pdf_gaussian_sample(pdf);
  pf_pdf_gaussian_free(pdf);
  return sample;
}

int main()
{
  const pf_vector_t first = draw(3407);
  const pf_vector_t repeat = draw(3407);
  const pf_vector_t other = draw(3408);
  if (!equal(first, repeat) || equal(first, other))
  {
    std::cerr << "explicit seed was overwritten by Gaussian PDF initialization" << std::endl;
    return 1;
  }
  pf_pdf_set_seed(3407);
  auto * pdf = makePdf();
  pf_pdf_gaussian_sample(pdf);
  const pf_vector_t second = pf_pdf_gaussian_sample(pdf);
  pf_pdf_gaussian_free(pdf);
  pf_pdf_set_seed(3407);
  pdf = makePdf();
  pf_pdf_gaussian_sample(pdf);
  pf_pdf_gaussian_free(pdf);
  pdf = makePdf();
  const pf_vector_t second_allocation = pf_pdf_gaussian_sample(pdf);
  pf_pdf_gaussian_free(pdf);
  if (!equal(second, second_allocation))
  {
    return 2;
  }
  if (equal(draw(-1), draw(-1)))
  {
    return 3;
  }
  return 0;
}
