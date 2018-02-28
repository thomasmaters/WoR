/*
 * EnumOperators.hpp
 *
 *  Created on: 26 dec. 2017
 *      Author: Thomas
 */

#ifndef ENUMOPERATORS_HPP_
#define ENUMOPERATORS_HPP_

/**
 * Bitwise enum OR operator.
 * @param lhs
 * @param rhs
 * @return
 * @author Thomas Maters
 */
template <typename Enum>
Enum operator|(Enum lhs, Enum rhs)
{
    using underlying = typename std::underlying_type<Enum>::type;
    return static_cast<Enum>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

/**
 * Bitwise enum AND operator.
 * @param lhs
 * @param rhs
 * @return
 * @author Thomas Maters
 */
template <typename Enum>
Enum operator&(Enum lhs, Enum rhs)
{
    using underlying = typename std::underlying_type<Enum>::type;
    return static_cast<Enum>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}

#endif /* ENUMOPERATORS_HPP_ */
