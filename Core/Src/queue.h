#ifndef USER_QUEUE
#define USER_QUEUE

#include <stdio.h>
#include <stdlib.h>


typedef struct node {
	int val;
	struct node *next;
} node_t;


void enqueue(node_t **head, int val) {
    node_t *new_node = malloc(sizeof(node_t));
    if (!new_node) return;

    new_node->val = val;
    new_node->next = *head;

    *head = new_node;
}

int dequeue(node_t **head) {
    node_t *current, *prev = NULL;
    int retval = -1;

    if (*head == NULL) return -1;

    current = *head;
    while (current->next != NULL) {
        prev = current;
        current = current->next;
    }

    retval = current->val;
    free(current);

    if (prev)
        prev->next = NULL;
    else
        *head = NULL;

    return retval;
}

int mean_queue(node_t *head, uint8_t size) {
    node_t *current = head;
    int sum = 0;
    int n = 0;
    while (current != NULL && n < size) {
        sum = sum + current->val;
        current = current->next;
        n++;
    }
    return sum/n;
}

void print_list(node_t *head) {
    node_t *current = head;

    while (current != NULL) {
        printf("%d\n", current->val);
        current = current->next;
    }
}

int example() {
    node_t *head = NULL;
    int ret;

    enqueue(&head, 11);
    enqueue(&head, 22);
    enqueue(&head, 33);
    enqueue(&head, 44);

    print_list(head);

    while ((ret=dequeue(&head)) > 0) {
        printf("dequeued %d\n", ret);
    }
    printf("done. head=%p\n", head);

    return 0;
}

#endif // USER_QUEUE
