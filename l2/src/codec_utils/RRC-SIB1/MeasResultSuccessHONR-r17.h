/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasResultSuccessHONR_r17_H_
#define	_MeasResultSuccessHONR_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct MeasQuantityResults;
struct ResultsPerSSB_IndexList;
struct ResultsPerCSI_RS_IndexList;

/* MeasResultSuccessHONR-r17 */
typedef struct MeasResultSuccessHONR_r17 {
	struct MeasResultSuccessHONR_r17__measResult_r17 {
		struct MeasResultSuccessHONR_r17__measResult_r17__cellResults_r17 {
			struct MeasQuantityResults	*resultsSSB_Cell_r17;	/* OPTIONAL */
			struct MeasQuantityResults	*resultsCSI_RS_Cell_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} cellResults_r17;
		struct MeasResultSuccessHONR_r17__measResult_r17__rsIndexResults_r17 {
			struct ResultsPerSSB_IndexList	*resultsSSB_Indexes_r17;	/* OPTIONAL */
			struct ResultsPerCSI_RS_IndexList	*resultsCSI_RS_Indexes_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} rsIndexResults_r17;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} measResult_r17;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasResultSuccessHONR_r17_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasResultSuccessHONR_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasResultSuccessHONR_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasResultSuccessHONR_r17_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasQuantityResults.h"
#include "ResultsPerSSB-IndexList.h"
#include "ResultsPerCSI-RS-IndexList.h"

#endif	/* _MeasResultSuccessHONR_r17_H_ */
#include <asn_internal.h>
