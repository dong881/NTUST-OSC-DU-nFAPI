/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_CapabilityRAT_ContainerList_H_
#define	_UE_CapabilityRAT_ContainerList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct UE_CapabilityRAT_Container;

/* UE-CapabilityRAT-ContainerList */
typedef struct UE_CapabilityRAT_ContainerList {
	A_SEQUENCE_OF(struct UE_CapabilityRAT_Container) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_CapabilityRAT_ContainerList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UE_CapabilityRAT_ContainerList;
extern asn_SET_OF_specifics_t asn_SPC_UE_CapabilityRAT_ContainerList_specs_1;
extern asn_TYPE_member_t asn_MBR_UE_CapabilityRAT_ContainerList_1[1];
extern asn_per_constraints_t asn_PER_type_UE_CapabilityRAT_ContainerList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UE-CapabilityRAT-Container.h"

#endif	/* _UE_CapabilityRAT_ContainerList_H_ */
#include <asn_internal.h>
