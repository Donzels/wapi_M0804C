/**
 * @file t_list.h
 * @brief Header for Intrusive Doubly-Linked Circular List Primitives
 *
 * Defines the data structure and API for an intrusive doubly-linked circular list.
 * Intrusive lists embed list nodes directly into user data structures (no extra wrapper),
 * optimizing memory usage and simplifying integration. A sentinel node is used to
 * handle edge cases (empty list, head/tail operations) uniformly.
 */

#ifndef __T_LIST_H__
#define __T_LIST_H__

#include <stdint.h>
#include <stdbool.h>

#define T_CONTAINER_OF(ptr, type, member)\
      ((type*)((char*)(ptr) - (unsigned long)(&((type*)0)->member)))

#define T_LIST_ENTRY(node, type, member)\
        T_CONTAINER_OF(node, type, member)    

typedef struct t_list
{
  struct t_list *next;
  struct t_list *pre; 
}t_list_t;

void t_list_init(t_list_t *node);
void t_list_insert_after(t_list_t *node, t_list_t *new);
void t_list_insert_before(t_list_t *node, t_list_t *new);
void t_list_remove(t_list_t *node);
bool t_list_is_empty(t_list_t *node);

#endif//__T_LIST_H__

#define CONTAINER_OF(ptr, type, member)\
            
