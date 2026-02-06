/**
 * @file t_list.c
 * @brief Intrusive Doubly-Linked Circular List Implementation
 *
 * Provides core primitives for an intrusive doubly-linked circular list,
 * where list nodes are embedded directly into user data structures (no separate wrapper).
 * The list uses a sentinel node to simplify edge-case handling (empty list, head/tail operations).
 */

#include "t_list.h"

void t_list_init(t_list_t *node)
{
  node->next = node->pre = node;
}

void t_list_insert_after(t_list_t *node, t_list_t *new)
{
  new->next = node->next;
  node->next->pre = new;
  node->next = new;
  new->pre = node;
}

void t_list_insert_before(t_list_t *node, t_list_t *new)
{
  new->next = node;
  node->pre->next = new;
  new->pre = node->pre;
  node->pre = new;
}

void t_list_remove(t_list_t *node)
{
  node->pre->next = node->next;
  node->next->pre = node->pre;
  t_list_init(node);
}

/* 1 if empty else is 0 */
bool t_list_is_empty(t_list_t *node)
{
  return node->next == node;
}
