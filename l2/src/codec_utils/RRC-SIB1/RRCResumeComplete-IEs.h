/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RRCResumeComplete_IEs_H_
#define	_RRCResumeComplete_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include "DedicatedNAS-Message.h"
#include <NativeInteger.h>
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct UplinkTxDirectCurrentList;
struct RRCResumeComplete_v1610_IEs;

/* RRCResumeComplete-IEs */
typedef struct RRCResumeComplete_IEs {
	DedicatedNAS_Message_t	*dedicatedNAS_Message;	/* OPTIONAL */
	long	*selectedPLMN_Identity;	/* OPTIONAL */
	struct UplinkTxDirectCurrentList	*uplinkTxDirectCurrentList;	/* OPTIONAL */
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	struct RRCResumeComplete_v1610_IEs	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RRCResumeComplete_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RRCResumeComplete_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_RRCResumeComplete_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_RRCResumeComplete_IEs_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UplinkTxDirectCurrentList.h"
#include "RRCResumeComplete-v1610-IEs.h"

#endif	/* _RRCResumeComplete_IEs_H_ */
#include <asn_internal.h>
